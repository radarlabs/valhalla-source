#include "baldr/graphreader.h"
#include "baldr/predictedspeeds.h"
#include "baldr/rapidjson_utils.h"
#include "filesystem.h"
#include "mjolnir/graphtilebuilder.h"
#include "rapidjson/document.h"

#include <cstdint>
#include <cxxopts.hpp>
#include <fcntl.h>

#include <algorithm>
#include <boost/algorithm/string/replace.hpp>
#include <boost/property_tree/ptree.hpp>

#include <fstream>
#include <iostream>

#include "config.h"
#include "microtar.h"

using namespace valhalla::baldr;
using namespace valhalla::midgard;

struct MMap {
  MMap(const char* filename) {
    fd = open(filename, O_RDWR);
    struct stat s;
    fstat(fd, &s);

    data = mmap(0, s.st_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    length = s.st_size;
  }

  ~MMap() {
    munmap(data, length);
    close(fd);
  }

  int fd;
  void* data;
  size_t length;
};

class MMapGraphMemory final : public valhalla::baldr::GraphMemory {
public:
  MMapGraphMemory(std::shared_ptr<MMap> mmap, char* data_, size_t size_) : mmap_(std::move(mmap)) {
    data = data_;
    size = size_;
  }

private:
  const std::shared_ptr<MMap> mmap_;
};

typedef struct {
  char name[100];
  char mode[8];
  char owner[8];
  char group[8];
  char size[12];
  char mtime[12];
  char checksum[8];
  char type;
  char linkname[100];
  char _padding[255];
} mtar_raw_header_t_;

struct EdgeAndDirection {
  bool forward;
  uint32_t length;
  GraphId tileid;
  GraphId edgeid;

  EdgeAndDirection(const bool f, uint32_t len, valhalla::baldr::GraphId graphid, const GraphId& id) : forward(f), length(len), tileid(graphid), edgeid(id) {
  }
};

struct TileSet {
    int level;
    double size;
};

const std::vector<TileSet> valhalla_tiles = {
    {2, 0.25},
    {1, 1.0},
    {0, 4.0}
};

// Constants
constexpr int LEVEL_BITS = 3;
constexpr int TILE_INDEX_BITS = 22;
constexpr int ID_INDEX_BITS = 21;

// Masks
constexpr uint32_t LEVEL_MASK = (1 << LEVEL_BITS) - 1;  // 0b111 (3 bits set)
constexpr uint32_t TILE_INDEX_MASK = (1 << TILE_INDEX_BITS) - 1;  // 22 bits set
constexpr uint32_t ID_INDEX_MASK = (1 << ID_INDEX_BITS) - 1;  // 21 bits set

// INVALID_ID calculation
constexpr uint64_t INVALID_ID = 
    (static_cast<uint64_t>(ID_INDEX_MASK) << (TILE_INDEX_BITS + LEVEL_BITS)) | 
    (static_cast<uint64_t>(TILE_INDEX_MASK) << LEVEL_BITS) | 
    LEVEL_MASK;

struct tile_index_entry {
  uint64_t offset;
  uint32_t tile_id;
  uint32_t size;
};

void update_tile_traffic(
  const boost::property_tree::ptree& config,
  uint64_t tile_offset,
  uint64_t traffic_update_timestamp,
  std::vector<std::string> traffic_params
) {
  const auto memory = std::make_shared<MMap>(config.get<std::string>("mjolnir.traffic_extract").c_str());

  mtar_t tar;
  tar.pos = tile_offset;
  tar.stream = memory->data;
  tar.read = [](mtar_t* tar, void* data, unsigned size) -> int {
    memcpy(data, reinterpret_cast<char*>(tar->stream) + tar->pos, size);
    return MTAR_ESUCCESS;
  };
  tar.write = [](mtar_t* tar, const void* data, unsigned size) -> int {
    memcpy(reinterpret_cast<char*>(tar->stream) + tar->pos, data, size);
    return MTAR_ESUCCESS;
  };
  tar.seek = [](mtar_t*, unsigned) -> int { return MTAR_ESUCCESS; };
  tar.close = [](mtar_t*) -> int { return MTAR_ESUCCESS; };

  // Read the tile header
  mtar_header_t tar_header;
  mtar_read_header(&tar, &tar_header);

  valhalla::baldr::TrafficTile tile(
      std::make_unique<MMapGraphMemory>(memory,
                                        reinterpret_cast<char*>(tar.stream) + tar.pos +
                                            sizeof(mtar_raw_header_t_),
                                        tar_header.size));

  for (int paramOffset = 2; paramOffset < traffic_params.size();) {
    uint64_t edge_index = static_cast<uint64_t>(std::stoul(traffic_params[paramOffset]));

    if (edge_index >= tile.header->directed_edge_count) {
      throw std::runtime_error("Edge index out of bounds");
    }

    // Access and update the edge's traffic data
    valhalla::baldr::TrafficSpeed* target_edge = const_cast<valhalla::baldr::TrafficSpeed*>(tile.speeds + edge_index);
    target_edge->overall_encoded_speed = static_cast<uint64_t>(std::stoul(traffic_params[paramOffset + 1]));
    target_edge->encoded_speed1 = static_cast<uint64_t>(std::stoul(traffic_params[paramOffset + 2]));
    target_edge->encoded_speed2 = static_cast<uint64_t>(std::stoul(traffic_params[paramOffset + 3]));
    target_edge->encoded_speed3 = static_cast<uint64_t>(std::stoul(traffic_params[paramOffset + 4]));
    target_edge->breakpoint1 = static_cast<uint64_t>(std::stoul(traffic_params[paramOffset + 5]));
    target_edge->breakpoint2 = static_cast<uint64_t>(std::stoul(traffic_params[paramOffset + 6]));
    target_edge->congestion1 = static_cast<uint64_t>(0);
    target_edge->congestion2 = static_cast<uint64_t>(0);
    target_edge->congestion3 = static_cast<uint64_t>(0);
    target_edge->has_incidents = static_cast<uint64_t>(0);

    paramOffset = paramOffset + 7;
  }

  tile.header->last_update = traffic_update_timestamp;
}

int handle_update_tile_traffic(
  cxxopts::ParseResult cmd_args,
  std::string config_file_path
) {
  boost::property_tree::ptree pt;
  if (cmd_args.count("config") && filesystem::is_regular_file(config_file_path)) {
    rapidjson::read_json(config_file_path, pt);
  } else {
    std::cerr << "Configuration is required" << std::endl;
    return EXIT_FAILURE;
  }

  std::vector<std::string> live_traffic_params = cmd_args["update-tile-traffic"].as<std::vector<std::string>>();

  uint64_t tile_offset = static_cast<uint64_t>(std::stoul(live_traffic_params[0]));
  uint64_t last_updated = static_cast<uint64_t>(std::stoul(live_traffic_params[1]));

  update_tile_traffic(pt, tile_offset, last_updated, live_traffic_params);
  std::cout << "Updated edge_id successfully at " << pt.get<std::string>("mjolnir.traffic_extract") << std::endl;
  return EXIT_SUCCESS;
}

// helper function for parsing the columns out of the CSV row
inline std::array<std::string, 8> split(const std::string& str, char delim) {
  std::array<std::string, 8> result;
  size_t pos = 0;
  size_t found;

  for (int i = 0; i < 7; i++) {  // Process first 7 fields (last one is handled separately)
    found = str.find(delim, pos);
    if (found == std::string::npos) {
      result[i] = str.substr(pos);
      return result;
    }
    result[i] = str.substr(pos, found - pos);
    pos = found + 1;
  }

  // Last field (everything after the 7th comma)
  result[7] = str.substr(pos);

  return result;
}

int handle_ingest_traffic_updates(
  cxxopts::ParseResult cmd_args,
  std::string config_file_path
) {

  // load vallhalla config json
	boost::property_tree::ptree config;
  if (cmd_args.count("config") && filesystem::is_regular_file(config_file_path)) {
    rapidjson::read_json(config_file_path, config);
  } else {
    std::cerr << "Configuration is required" << std::endl;
    return EXIT_FAILURE;
  }

  // input file
  std::string traffic_update_file_path = cmd_args["ingest-tile-traffic"].as<std::string>();
  std::cout << "Opening file: " << traffic_update_file_path << std::endl;

  // open and memory map the file
  int fd = open(traffic_update_file_path.c_str(), O_RDONLY);
  if (fd == -1) {
    std::cerr << "Cannot open file" << std::endl;
    return 1;
  }

  struct stat sb;
  if (fstat(fd, &sb) == -1) {
    std::cerr << "Cannot get file size" << std::endl;
    close(fd);
    return 1;
  }

  char* addr = static_cast<char*>(mmap(nullptr, sb.st_size, PROT_READ, MAP_PRIVATE, fd, 0));
  if (addr == MAP_FAILED) {
    std::cerr << "Cannot map file" << std::endl;
    close(fd);
    return 1;
  }

  // setup MMAP to traffic file
  const auto memory = std::make_shared<MMap>(config.get<std::string>("mjolnir.traffic_extract").c_str());
  mtar_t tar;
  unsigned current_offset = 0;
  std::unique_ptr<valhalla::baldr::TrafficTile> tile;

  // process lines
  char* end = addr + sb.st_size;
  char* lineStart = addr;

  int count = 0;
  for (char* p = addr; p != end; ++p) {
    if (*p == '\n') {
      std::string line(lineStart, p - lineStart);

      auto fields = split(line, ',');

      // first column is the tile_offset - rows are grouped by tile_offset
      uint64_t tile_offset = std::stol(fields[0]);
      uint64_t edge_index = std::stol(fields[1]);

      // update the current MMAP'ed tile
      if (tile_offset != current_offset) {
        tar.pos = tile_offset;
        tar.stream = memory->data;
        tar.read = [](mtar_t* tar, void* data, unsigned size) -> int {
          memcpy(data, reinterpret_cast<char*>(tar->stream) + tar->pos, size);
          return MTAR_ESUCCESS;
        };
        tar.write = [](mtar_t* tar, const void* data, unsigned size) -> int {
          memcpy(reinterpret_cast<char*>(tar->stream) + tar->pos, data, size);
          return MTAR_ESUCCESS;
        };
        tar.seek = [](mtar_t*, unsigned) -> int { return MTAR_ESUCCESS; };
        tar.close = [](mtar_t*) -> int { return MTAR_ESUCCESS; };

        // read the tile header
        mtar_header_t tar_header;
        mtar_read_header(&tar, &tar_header);

        tile = std::make_unique<valhalla::baldr::TrafficTile>(std::make_unique<MMapGraphMemory>(
          memory,
          reinterpret_cast<char*>(tar.stream) + tar.pos + sizeof(mtar_raw_header_t_),
          tar_header.size
        ));

        current_offset = tile_offset;
      }

      // access and update the edge's traffic data
      valhalla::baldr::TrafficSpeed* target_edge = const_cast<valhalla::baldr::TrafficSpeed*>(tile->speeds + edge_index);
      target_edge->overall_encoded_speed = std::stol(fields[2]);
      target_edge->encoded_speed1 = std::stol(fields[3]);
      target_edge->encoded_speed2 = std::stol(fields[4]);
      target_edge->encoded_speed3 = std::stol(fields[5]);
      target_edge->breakpoint1 = std::stol(fields[6]);
      target_edge->breakpoint2 = std::stol(fields[7]);
      target_edge->congestion1 = 0;
      target_edge->congestion2 = 0;
      target_edge->congestion3 = 0;
      target_edge->has_incidents = 0;

      lineStart = p + 1; // move to next line
                         //
      count++;
      if (count & 100000) {
        std::cout << "Updated " << count << "edges" << std::endl;
      }
    }
  }

  // Clean up
  munmap(addr, sb.st_size);
  close(fd);

  std::cout << "Processed " << count << " lines" << std::endl;

  return EXIT_SUCCESS;
}

int handle_ways_to_edges(
  std::string config_file_path
) {
  // load valhalla config json
	boost::property_tree::ptree config;
  if (filesystem::is_regular_file(config_file_path)) {
    rapidjson::read_json(config_file_path, config);
  } else {
    std::cerr << "Configuration is required" << std::endl;
    return EXIT_FAILURE;
  }

  std::unordered_map<uint64_t, std::vector<EdgeAndDirection>> ways_edges;

  GraphReader reader(config.get_child("mjolnir"));
  for (auto edge_id : reader.GetTileSet()) {
    if (!reader.DoesTileExist(edge_id)) {
      continue;
    }
    if (reader.OverCommitted()) {
      reader.Trim();
    }

    graph_tile_ptr tile = reader.GetGraphTile(edge_id);
    for (uint32_t n = 0; n < tile->header()->directededgecount(); n++, ++edge_id) {
      const DirectedEdge* edge = tile->directededge(edge_id);

      // skip if transit or shortcut
      if (edge->IsTransitLine() || edge->use() == Use::kTransitConnection ||
          edge->use() == Use::kEgressConnection || edge->use() == Use::kPlatformConnection ||
          edge->is_shortcut()) {
        continue;
      }

      // skip if not auto accessible
      if (!(edge->forwardaccess() & kAutoAccess)) {
        continue;
      }

      valhalla::baldr::GraphId graph_id(edge_id);

      uint64_t wayid = tile->edgeinfo(edge).wayid();
      ways_edges[wayid].push_back({edge->forward(), edge->length(), graph_id, edge_id});
    }
  }

  std::ofstream ways_file;
  std::string fname = config.get<std::string>("mjolnir.tile_dir") +
                      filesystem::path::preferred_separator + "way_edges_traffic.csv";
  ways_file.open(fname, std::ofstream::out | std::ofstream::trunc);
  for (const auto& way : ways_edges) {
    ways_file << way.first;
    bool isFirst = true;
    for (auto edge : way.second) {
      if (isFirst) {
        ways_file << "," << (uint32_t)edge.forward << ";" << edge.length << ";" << edge.tileid << ";" << (uint64_t)edge.edgeid;
        isFirst = false;
      }
      else {
        ways_file << ";" << (uint32_t)edge.forward << ";" << edge.length << ";" << edge.tileid << ";" << (uint64_t)edge.edgeid;
      }
    }
    ways_file << std::endl;
  }
  ways_file.close();

  LOG_INFO("Finished with " + std::to_string(ways_edges.size()) + " ways.");

  return EXIT_SUCCESS;
}

std::unordered_map<std::string, uint64_t> traffic_tiles;

// callback to process the traffic tar extract
// - only read the index.bin file, which contains the offsets for each tile
auto index_loader = [](
  const std::string& filename,
  const char* index_begin,
  const char* file_begin,
  size_t size
) -> decltype(valhalla::midgard::tar::contents) {
  if (filename != "index.bin")
      return {};

  decltype(valhalla::midgard::tar::contents) contents;

  // iterate over index entries
  auto entries = valhalla::midgard::iterable_t<tile_index_entry>(
    reinterpret_cast<tile_index_entry*>(const_cast<char*>(index_begin)),
    size / sizeof(tile_index_entry)
  );

  // store tile_id -> offset in "traffic_tiles"
  for (const auto& entry : entries) {
    valhalla::baldr::GraphId graph_id(entry.tile_id);
    traffic_tiles.emplace(std::to_string(graph_id), entry.offset - 512);
  }

  return contents;
};

// create tile_id to traffic.tar offset mapping
int handle_tile_offset_index(
  std::string config_file_path
) {

  // load valhalla config json
  boost::property_tree::ptree config;
  if (filesystem::is_regular_file(config_file_path)) {
    rapidjson::read_json(config_file_path, config);
  } else {
    std::cerr << "Configuration is required" << std::endl;
    return EXIT_FAILURE;
  }

  // load the traffic.tar archive to create the tile_id -> offset index
  std::unique_ptr<valhalla::midgard::tar> archive(new valhalla::midgard::tar(
    config.get<std::string>("mjolnir.traffic_extract"),
    true,
    true,
    index_loader
  ));
  std::cout << "Loaded index from .tar archive." << std::endl;

  // write out the tar file offsets for each tile_id
  std::ofstream tar_index_file;
  std::string fname = config.get<std::string>("mjolnir.tile_dir") + filesystem::path::preferred_separator + "traffic_tile_offset.csv";
  tar_index_file.open(fname, std::ofstream::out | std::ofstream::trunc);
  for (const auto& index : traffic_tiles) {
    tar_index_file << index.first << "," << index.second << std::endl;
  }
  tar_index_file.close();

  LOG_INFO("Finished with " + std::to_string(traffic_tiles.size()) + " ways.");

  return EXIT_SUCCESS;
}

int handle_help(cxxopts::Options options) {
  std::cout << options.help() << std::endl;
  return EXIT_SUCCESS;
}

int main(int argc, char** argv) {
  // args
  std::string config_file_path;
  try {
    // clang-format off
    cxxopts::Options options(argv[0], " - Provides utilities for adding traffic to valhalla routing tiles.");

    options.add_options()
        ("h,help", "Print this help message.")
        ("c,config", "Path to the json configuration file.",
            cxxopts::value<std::string>(config_file_path))
				("update-tile-traffic", "Update traffic for a given tile. Usage: --update-tile-traffic <tile_offset>,<timestamp>,[<edge_index>,<overall_speed>,<speed1>,<speed2>,<speed3>,<breakpoint1>,<breakpoint2>,<congestion1>,<congestion2>,<congestion3>,<has_incidents>,...]",
            cxxopts::value<std::vector<std::string>>())
				("ingest-tile-traffic", "Ingest all traffic updates from a CSV file. Usage: --update-tile-traffic <csv_file>",
            cxxopts::value<std::string>())
        ("ways-to-edges", "Creates a list of edges for each OSM way with some additional attributes")
        ("tile-offset-index", "Creates an index of tile name with their offset in traffic_extract file");

    // clang-format on

    auto cmd_args = options.parse(argc, argv);

    if (cmd_args.count("help")) {
      return handle_help(options);
    }

    if (cmd_args.count("ways-to-edges")) {
      return handle_ways_to_edges(config_file_path);
    }

    if (cmd_args.count("tile-offset-index")) {
      return handle_tile_offset_index(config_file_path);
    }

		if (cmd_args.count("update-tile-traffic")) {
      return handle_update_tile_traffic(cmd_args, config_file_path);
    }

		if (cmd_args.count("ingest-tile-traffic")) {
      return handle_ingest_traffic_updates(cmd_args, config_file_path);
    }

    std::cout << options.help() << std::endl;

  } catch (const std::exception& e) {
    std::cerr << "Unable to parse command line options because: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }
  return EXIT_FAILURE;
}
