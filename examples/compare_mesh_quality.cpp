// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// CPU-only geometry comparison. A fine uniform reconstruction is a proxy,
// not ground truth. Queries sample surface area rather than vertex density.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include <tinyply.h>

#include "volumetric_kit/recon/eval/mesh_distance.hpp"

namespace vr = volumetric_kit::recon;
namespace mesh = vr::mesh;
namespace eval = vr::eval;

namespace {
struct Options {
  std::string reference, test, cells_csv, misses_csv;
  std::size_t samples = 200000;
  float reach = 0.04f, threshold = 0.005f, cell = 0.1f;
  bool roi = false;
  std::array<float, 6> bounds{};
};

mesh::Mesh read_ply(const std::string& path) {
  std::ifstream stream(path, std::ios::binary);
  if (!stream) throw std::runtime_error("cannot open PLY: " + path);
  tinyply::PlyFile ply;
  if (!ply.parse_header(stream))
    throw std::runtime_error("invalid PLY header: " + path);
  const auto positions =
      ply.request_properties_from_element("vertex", {"x", "y", "z"});
  const auto faces =
      ply.request_properties_from_element("face", {"vertex_indices"}, 3);
  ply.read(stream);
  if (positions->t != tinyply::Type::FLOAT32 ||
      (faces->t != tinyply::Type::INT32 && faces->t != tinyply::Type::UINT32) ||
      positions->buffer.size_bytes() != positions->count * 12 ||
      faces->buffer.size_bytes() != faces->count * 12)
    throw std::runtime_error(
        "expected float32 positions and int32 triangle faces: " + path);
  mesh::Mesh result;
  result.vertices.resize(positions->count);
  result.indices.resize(faces->count * 3);
  for (std::size_t i = 0; i < positions->count; ++i)
    std::memcpy(&result.vertices[i].position, positions->buffer.get() + 12 * i,
                12);
  std::memcpy(result.indices.data(), faces->buffer.get(),
              faces->buffer.size_bytes());
  // Validate before sampling uses indices; MeshDistance also validates these
  // inputs, but is intentionally created only after samples are prepared.
  for (auto index : result.indices)
    if (index >= result.vertices.size())
      throw std::runtime_error("PLY index out of range");
  for (const auto& vertex : result.vertices) {
    const auto p = vertex.position;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
      throw std::runtime_error("non-finite PLY position");
  }
  return result;
}

std::uint64_t mix(std::uint64_t value) {
  value += 0x9e3779b97f4a7c15ULL;
  value = (value ^ (value >> 30)) * 0xbf58476d1ce4e5b9ULL;
  value = (value ^ (value >> 27)) * 0x94d049bb133111ebULL;
  return value ^ (value >> 31);
}
double random_unit(std::uint64_t value) {
  return static_cast<double>(mix(value) >> 11) * 0x1.0p-53;
}

struct Triangle {
  std::array<vr::Vec3f, 3> p;
  vr::Vec3f normal;
  double area = 0;
  std::uint64_t seed = 0;
};
Triangle triangle(const mesh::Mesh& m, std::size_t i) {
  Triangle t;
  for (std::size_t j = 0; j < 3; ++j)
    t.p[j] = m.vertices[m.indices[i + j]].position;
  // Canonical corners make the seed and barycentric locations independent of
  // triangle emission order and winding. Normal strata are sign invariant.
  std::sort(t.p.begin(), t.p.end(), [](vr::Vec3f a, vr::Vec3f b) {
    return std::array<float, 3>{a.x, a.y, a.z} <
           std::array<float, 3>{b.x, b.y, b.z};
  });
  const auto a = t.p[1] - t.p[0], b = t.p[2] - t.p[0];
  const double x = double(a.y) * b.z - double(a.z) * b.y;
  const double y = double(a.z) * b.x - double(a.x) * b.z;
  const double z = double(a.x) * b.y - double(a.y) * b.x;
  const double length = std::sqrt(x * x + y * y + z * z);
  t.area = length * 0.5;
  if (length > 0)
    t.normal =
        vr::Vec3f(float(x / length), float(y / length), float(z / length));
  t.seed = 0x7265636f6e202610ULL;
  for (const auto p : t.p) {
    for (float coordinate : {p.x, p.y, p.z}) {
      if (coordinate == 0) coordinate = 0;  // normalize signed zero in seed
      std::uint32_t bits;
      std::memcpy(&bits, &coordinate, sizeof(bits));
      t.seed = mix(t.seed ^ bits);
    }
  }
  return t;
}

struct Sample {
  vr::Vec3f position, normal;
};
bool inside(vr::Vec3f p, const Options& o) {
  return !o.roi ||
         (p.x >= o.bounds[0] && p.y >= o.bounds[1] && p.z >= o.bounds[2] &&
          p.x <= o.bounds[3] && p.y <= o.bounds[4] && p.z <= o.bounds[5]);
}
std::vector<Sample> sample_surface(const mesh::Mesh& m, const Options& opt) {
  double area = 0;
  for (std::size_t i = 0; i < m.indices.size(); i += 3)
    area += triangle(m, i).area;
  if (!(area > 0) || !std::isfinite(area))
    throw std::runtime_error("mesh area must be finite and positive");
  std::vector<Sample> samples;
  samples.reserve(opt.samples);
  const double density = static_cast<double>(opt.samples) / area;
  for (std::size_t i = 0; i < m.indices.size(); i += 3) {
    const auto t = triangle(m, i);
    const double expected = t.area * density;
    const auto whole = static_cast<std::size_t>(std::floor(expected));
    // Unbiased stochastic rounding gives approximately --samples queries.
    // Seeds depend on geometry, so GPU emission order cannot change a run.
    const auto count =
        whole + (random_unit(t.seed) < expected - whole ? 1u : 0u);
    for (std::size_t j = 0; j < count; ++j) {
      const double u = std::sqrt(random_unit(t.seed + 2 * j + 1));
      const double v = random_unit(t.seed + 2 * j + 2);
      const auto p = t.p[0] * float(1 - u) + t.p[1] * float(u * (1 - v)) +
                     t.p[2] * float(u * v);
      if (inside(p, opt)) samples.push_back({p, t.normal});
    }
  }
  return samples;
}

using Cell = std::array<std::int64_t, 3>;
Cell cell_for(vr::Vec3f p, float size) {
  for (float coordinate : {p.x, p.y, p.z})
    if (std::abs(double(coordinate) / size) >=
        double(std::numeric_limits<std::int64_t>::max()) / 2)
      throw std::runtime_error("--cell is too small for the mesh coordinates");
  return {static_cast<std::int64_t>(std::floor(double(p.x) / size)),
          static_cast<std::int64_t>(std::floor(double(p.y) / size)),
          static_cast<std::int64_t>(std::floor(double(p.z) / size))};
}
struct CellStats {
  std::array<double, 9> moments{};
  std::size_t normals = 0;
  int stratum =
      3;  // 0 planar, 1 intermediate, 2 detail, 3 insufficient evidence
  std::vector<float> accuracy, coverage;
};
constexpr std::array<const char*, 4> kStrata = {"planar", "intermediate",
                                                "detail", "unknown"};

void classify_cell(CellStats& cell) {
  if (cell.normals < 8) return;
  std::array<double, 3> axis{};
  int largest = 0;
  for (int i = 1; i < 3; ++i)
    if (cell.moments[4 * i] > cell.moments[4 * largest]) largest = i;
  axis[largest] = 1;
  double eigen = 0;
  for (int step = 0; step < 20; ++step) {
    std::array<double, 3> next{};
    for (int row = 0; row < 3; ++row)
      for (int column = 0; column < 3; ++column)
        next[row] += cell.moments[3 * row + column] * axis[column];
    eigen =
        std::sqrt(next[0] * next[0] + next[1] * next[1] + next[2] * next[2]);
    if (!(eigen > 0)) return;
    for (int row = 0; row < 3; ++row) axis[row] = next[row] / eigen;
  }
  // Largest eigenvalue of E[n n^T]: sign invariant coherence, thresholds
  // cos(10deg)^2 and cos(25deg)^2. These are geometric strata, not semantics.
  const double coherence = eigen / static_cast<double>(cell.normals);
  cell.stratum = coherence >= 0.9698463104   ? 0
                 : coherence <= 0.8213938048 ? 2
                                             : 1;
}

double fraction_within(const std::vector<float>& values, float threshold) {
  if (values.empty()) return 0;
  return static_cast<double>(std::count_if(
             values.begin(), values.end(),
             [&](float distance) { return distance <= threshold; })) /
         values.size();
}
void print_stats(const char* name, const std::vector<float>& accuracy,
                 const std::vector<float>& coverage, const Options& opt) {
  const auto a = eval::summarize(accuracy, opt.reach);
  const auto c = eval::summarize(coverage, opt.reach);
  const double precision = fraction_within(accuracy, opt.threshold);
  const double recall = fraction_within(coverage, opt.threshold);
  const double score = precision + recall > 0
                           ? 2 * precision * recall / (precision + recall)
                           : 0;
  std::printf(
      "%s: accuracy n=%zu mean %.6f p95 %.6f mm beyond %zu; "
      "coverage n=%zu mean %.6f p95 %.6f mm beyond %zu; "
      "precision %.6f recall %.6f F %.6f\n",
      name, a.count, 1000 * a.mean, 1000 * a.p95, a.beyond_reach, c.count,
      1000 * c.mean, 1000 * c.p95, c.beyond_reach, precision, recall, score);
}

void compare(const Options& opt) {
  auto reference = read_ply(opt.reference);
  auto ref_samples = sample_surface(reference, opt);
  auto indexed_reference = eval::MeshDistance::create(reference, opt.reach);
  if (!indexed_reference)
    throw std::runtime_error(indexed_reference.status().message());
  reference = {};  // release the dense vertex payload before loading test
  auto test = read_ply(opt.test);
  const auto test_samples = sample_surface(test, opt);
  auto indexed_test = eval::MeshDistance::create(test, opt.reach);
  if (!indexed_test) throw std::runtime_error(indexed_test.status().message());
  test = {};
  std::map<Cell, CellStats> cells;
  for (const auto& sample : ref_samples) {
    auto& cell = cells[cell_for(sample.position, opt.cell)];
    ++cell.normals;
    const std::array<double, 3> normal = {sample.normal.x, sample.normal.y,
                                          sample.normal.z};
    for (int row = 0; row < 3; ++row)
      for (int column = 0; column < 3; ++column)
        cell.moments[3 * row + column] += normal[row] * normal[column];
  }
  for (auto& entry : cells) classify_cell(entry.second);
  std::ofstream misses;
  if (!opt.misses_csv.empty()) {
    misses.open(opt.misses_csv);
    if (!misses) throw std::runtime_error("cannot open missed-sample CSV");
    misses << "x,y,z,nx,ny,nz,distance_m\n" << std::setprecision(9);
  }
  for (const auto& sample : ref_samples) {
    const float distance = indexed_test.value().distance(sample.position);
    cells[cell_for(sample.position, opt.cell)].coverage.push_back(distance);
    if (misses.is_open() && distance >= opt.reach)
      misses << sample.position.x << ',' << sample.position.y << ','
             << sample.position.z << ',' << sample.normal.x << ','
             << sample.normal.y << ',' << sample.normal.z << ',' << distance
             << '\n';
  }
  if (misses.is_open()) {
    misses.close();
    if (!misses) throw std::runtime_error("failed to write missed-sample CSV");
  }
  for (const auto& sample : test_samples)
    cells[cell_for(sample.position, opt.cell)].accuracy.push_back(
        indexed_reference.value().distance(sample.position));
  std::array<std::vector<float>, 4> accuracy, coverage;
  std::vector<float> all_accuracy, all_coverage;
  std::ofstream csv;
  if (!opt.cells_csv.empty()) {
    csv.open(opt.cells_csv);
    if (!csv) throw std::runtime_error("cannot open cell CSV");
    csv << "cell_x,cell_y,cell_z,stratum,accuracy_samples,coverage_samples,"
           "accuracy_mean_mm,accuracy_p95_mm,accuracy_beyond_reach,coverage_"
           "mean_mm,coverage_p95_mm,coverage_beyond_reach\n";
    csv << std::fixed << std::setprecision(6);
  }
  for (const auto& entry : cells) {
    const auto& c = entry.second;
    accuracy[c.stratum].insert(accuracy[c.stratum].end(), c.accuracy.begin(),
                               c.accuracy.end());
    coverage[c.stratum].insert(coverage[c.stratum].end(), c.coverage.begin(),
                               c.coverage.end());
    all_accuracy.insert(all_accuracy.end(), c.accuracy.begin(),
                        c.accuracy.end());
    all_coverage.insert(all_coverage.end(), c.coverage.begin(),
                        c.coverage.end());
    if (csv.is_open()) {
      const auto a = eval::summarize(c.accuracy, opt.reach),
                 r = eval::summarize(c.coverage, opt.reach);
      csv << entry.first[0] << ',' << entry.first[1] << ',' << entry.first[2]
          << ',' << kStrata[c.stratum] << ',' << a.count << ',' << r.count
          << ',' << a.mean * 1000 << ',' << a.p95 * 1000 << ','
          << a.beyond_reach << ',' << r.mean * 1000 << ',' << r.p95 * 1000
          << ',' << r.beyond_reach << '\n';
    }
  }
  if (csv.is_open()) {
    csv.close();
    if (!csv) throw std::runtime_error("failed to write cell CSV");
  }
  std::printf("reference proxy: %s\ntest: %s\n", opt.reference.c_str(),
              opt.test.c_str());
  if (opt.roi)
    std::printf("query ROI: [%.6f, %.6f, %.6f] to [%.6f, %.6f, %.6f] metres\n",
                opt.bounds[0], opt.bounds[1], opt.bounds[2], opt.bounds[3],
                opt.bounds[4], opt.bounds[5]);
  std::printf(
      "area samples requested %zu per mesh; reach %.6fm, F threshold %.6fm, "
      "reference cells %.6fm\n"
      "strata: normal coherence >=cos(10deg)^2 planar, <=cos(25deg)^2 detail, "
      ">=8 reference samples/cell\n"
      "mean/p95 exclude beyond-reach queries; F includes them as misses; proxy "
      "agreement is not ground-truth accuracy\n",
      opt.samples, opt.reach, opt.threshold, opt.cell);
  print_stats("all", all_accuracy, all_coverage, opt);
  for (std::size_t i = 0; i < kStrata.size(); ++i)
    print_stats(kStrata[i], accuracy[i], coverage[i], opt);
}

Options parse(int argc, char** argv) {
  Options opt;
  auto number = [&](int& i) {
    if (++i >= argc) throw std::runtime_error("missing numeric argument");
    std::size_t used = 0;
    const double result = std::stod(argv[i], &used);
    if (used != std::strlen(argv[i]) || !std::isfinite(result))
      throw std::runtime_error("invalid numeric argument");
    return result;
  };
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--samples") {
      const double n = number(i);
      if (n < 1 || n > 100000000 || std::floor(n) != n)
        throw std::runtime_error("invalid sample count");
      opt.samples = static_cast<std::size_t>(n);
    } else if (arg == "--reach")
      opt.reach = float(number(i));
    else if (arg == "--threshold")
      opt.threshold = float(number(i));
    else if (arg == "--cell")
      opt.cell = float(number(i));
    else if (arg == "--roi") {
      opt.roi = true;
      for (auto& bound : opt.bounds) bound = float(number(i));
    } else if (arg == "--cells-csv") {
      if (++i >= argc) throw std::runtime_error("missing CSV path");
      opt.cells_csv = argv[i];
    } else if (arg == "--misses-csv") {
      if (++i >= argc) throw std::runtime_error("missing CSV path");
      opt.misses_csv = argv[i];
    } else if (!arg.empty() && arg[0] == '-')
      throw std::runtime_error("unknown flag: " + arg);
    else if (opt.reference.empty())
      opt.reference = arg;
    else if (opt.test.empty())
      opt.test = arg;
    else
      throw std::runtime_error("unexpected argument: " + arg);
  }
  if (opt.reference.empty() || opt.test.empty())
    throw std::runtime_error(
        "usage: compare_mesh_quality reference.ply test.ply [--samples 200000] "
        "[--reach .04] [--threshold .005] [--cell .1] [--cells-csv path] "
        "[--misses-csv path] "
        "[--roi xmin ymin zmin xmax ymax zmax]");
  if (!std::isfinite(opt.reach) || !std::isfinite(opt.threshold) ||
      !std::isfinite(opt.cell) || !(opt.reach > 0) || !(opt.threshold > 0) ||
      opt.threshold >= opt.reach || !(opt.cell > 0))
    throw std::runtime_error(
        "positive finite cell/threshold/reach required; threshold must be less "
        "than reach");
  for (int i = 0; i < 3; ++i)
    if (opt.roi &&
        (!std::isfinite(opt.bounds[i]) || !std::isfinite(opt.bounds[i + 3]) ||
         opt.bounds[i] > opt.bounds[i + 3]))
      throw std::runtime_error("invalid ROI bounds");
  for (const auto& output : {opt.cells_csv, opt.misses_csv}) {
    if (output.empty()) continue;
    for (const auto& input : {opt.reference, opt.test}) {
      std::error_code csv_error, input_error, same_error;
      const auto csv = std::filesystem::weakly_canonical(output, csv_error);
      const auto path = std::filesystem::weakly_canonical(input, input_error);
      if ((!csv_error && !input_error && csv == path) ||
          std::filesystem::equivalent(output, input, same_error))
        throw std::runtime_error(
            "CSV outputs must differ from both input meshes");
    }
  }
  if (!opt.cells_csv.empty() && !opt.misses_csv.empty()) {
    std::error_code a, b, same;
    const auto cells = std::filesystem::weakly_canonical(opt.cells_csv, a);
    const auto missed = std::filesystem::weakly_canonical(opt.misses_csv, b);
    if ((!a && !b && cells == missed) ||
        std::filesystem::equivalent(opt.cells_csv, opt.misses_csv, same))
      throw std::runtime_error("CSV output paths must differ");
  }
  return opt;
}

int self_test() {
  mesh::Mesh plane;
  plane.vertices.resize(4);
  plane.vertices[0].position = vr::Vec3f(0, 0, 0);
  plane.vertices[1].position = vr::Vec3f(1, 0, 0);
  plane.vertices[2].position = vr::Vec3f(1, 1, 0);
  plane.vertices[3].position = vr::Vec3f(0, 1, 0);
  plane.indices = {0, 1, 2, 0, 2, 3};
  Options opt;
  opt.samples = 1000;
  opt.reach = 0.5f;  // the synthetic triangles span a metre, unlike room cells
  const auto points = sample_surface(plane, opt);
  if (points.size() != 1000) return 1;
  for (auto& v : plane.vertices) v.position.z = .003f;
  auto surface = eval::MeshDistance::create(plane, opt.reach);
  if (!surface) return 1;
  CellStats cell;
  for (const auto& p : points) {
    if (std::abs(surface.value().distance(p.position) - .003f) > 1e-6f)
      return 1;
    ++cell.normals;
    cell.moments[8] += 1;
  }
  classify_cell(cell);
  if (cell.stratum != 0) return 1;
  cell.moments = {500, 0, 0, 0, 0, 0, 0, 0, 500};
  classify_cell(cell);
  if (cell.stratum != 2) return 1;
  plane.indices = {3, 2, 0, 2, 1, 0};
  for (auto& v : plane.vertices) v.position.z = 0;
  const auto permuted = sample_surface(plane, opt);
  std::vector<std::array<float, 3>> a, b;
  for (const auto& p : points)
    a.push_back({p.position.x, p.position.y, p.position.z});
  for (const auto& p : permuted)
    b.push_back({p.position.x, p.position.y, p.position.z});
  std::sort(a.begin(), a.end());
  std::sort(b.begin(), b.end());
  return a == b ? 0 : 1;
}
}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc == 2 && std::string(argv[1]) == "--self-test") return self_test();
    const auto options = parse(argc, argv);
    compare(options);
  } catch (const std::exception& error) {
    std::fprintf(stderr, "compare_mesh_quality: %s\n", error.what());
    return 1;
  }
  return 0;
}
