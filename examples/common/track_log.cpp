// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "track_log.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>
#include <system_error>
#include <utility>

#include "volumetric_kit/recon/version.hpp"

namespace vr_example {

namespace {

using volumetric_kit::recon::Mat4f;
using volumetric_kit::recon::Vec3f;
using volumetric_kit::recon::Vec4f;

constexpr const char* kFormat = "volumetric_kit.track_log";
constexpr int kVersion = 1;
constexpr double kPi = 3.14159265358979323846;

// --- Names ------------------------------------------------------------------

const char* from_name(track::PoseFrom f) {
  switch (f) {
    case track::PoseFrom::Device:
      return "device";
    case track::PoseFrom::Anchored:
      return "anchored";
    case track::PoseFrom::Interpolated:
      return "interpolated";
    case track::PoseFrom::Coasting:
      return "coasting";
  }
  return "?";
}

const char* state_name(track::TrackState s) {
  switch (s) {
    case track::TrackState::Anchored:
      return "anchored";
    case track::TrackState::Partial:
      return "partial";
    case track::TrackState::Coasting:
      return "coasting";
    case track::TrackState::Rejected:
      return "rejected";
  }
  return "?";
}

std::optional<track::PoseFrom> parse_from(const std::string& s) {
  for (const track::PoseFrom f :
       {track::PoseFrom::Device, track::PoseFrom::Anchored,
        track::PoseFrom::Interpolated, track::PoseFrom::Coasting}) {
    if (s == from_name(f)) return f;
  }
  return std::nullopt;
}

std::optional<track::TrackState> parse_state(const std::string& s) {
  for (const track::TrackState t :
       {track::TrackState::Anchored, track::TrackState::Partial,
        track::TrackState::Coasting, track::TrackState::Rejected}) {
    if (s == state_name(t)) return t;
  }
  return std::nullopt;
}

std::optional<std::optional<track::FitKind>> parse_kind(const std::string& s) {
  if (s == "none") return std::optional<track::FitKind>{};
  for (const track::FitKind k : track::kFitKinds) {
    if (s == track::to_string(k)) return std::optional<track::FitKind>(k);
  }
  return std::nullopt;
}

// --- Writing ----------------------------------------------------------------

// A float in a form that reads back to the same float, a double to the same
// double.
std::string num(float v) {
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.9g", double(v));
  return buf;
}
std::string num(double v) {
  char buf[40];
  std::snprintf(buf, sizeof(buf), "%.17g", v);
  return buf;
}
std::string num(std::uint64_t v) { return std::to_string(v); }

template <typename Mat>
std::string mat(const Mat& m) {
  std::string s;
  for (int c = 0; c < 4; ++c) {
    for (int r = 0; r < 4; ++r) {
      if (c != 0 || r != 0) s += ',';
      s += num(m[c][r]);
    }
  }
  return s;
}

std::string mat_header(const char* name) {
  std::string s;
  for (int i = 0; i < 16; ++i) {
    if (i != 0) s += ',';
    s += std::string(name) + std::to_string(i);
  }
  return s;
}

// One line, the rest of it free text: no line breaks.
std::string one_line(std::string s) {
  std::replace(s.begin(), s.end(), '\n', ' ');
  std::replace(s.begin(), s.end(), '\r', ' ');
  return s;
}

vkc::Status write_text(const std::filesystem::path& path,
                       const std::string& text) {
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  out << text;
  out.close();
  if (!out) {
    return vkc::Status::io_error("TrackLog: cannot write " + path.string());
  }
  return {};
}

std::string run_csv(const TrackRun& run) {
  std::string s = "key,value\n";
  s += std::string("format,") + kFormat + "\n";
  s += "version," + std::to_string(kVersion) + "\n";
  s += "session," + one_line(run.source.session) + "\n";
  s += "registration," + one_line(run.source.registration) + "\n";
  s += "device," + one_line(run.source.device) + "\n";
  s += std::string("recon,") + volumetric_kit::recon::version_string() + "\n";
  s += "hold_ns," + num(run.hold_ns) + "\n";
  s += "tracking_to_world," + mat(run.source.tracking_to_world) + "\n";
  return s;
}

std::string chains_csv(const TrackRun& run) {
  std::string s = "name,role,fit,window_ns,hop_ns,passes," +
                  mat_header("initial_correction") + ",seed\n";
  for (const ChainInfo& c : run.chains) {
    s += c.name + (c.driver ? ",driver," : ",shadow,") +
         (c.fit ? track::to_string(*c.fit) : "none") + "," + num(c.window_ns) +
         "," + num(c.hop_ns) + "," + std::to_string(c.passes) + "," +
         mat(c.initial_correction) + "," + one_line(c.seed) + "\n";
  }
  return s;
}

std::string steps_csv(const TrackRun& run) {
  std::string s = "pts_ns," + mat_header("raw_from_sent") + "\n";
  for (const DeviceStep& d : run.source.device_steps) {
    s += num(d.pts_ns) + "," + mat(d.raw_from_sent) + "\n";
  }
  return s;
}

std::string frame_line(const FrameRecord& f) {
  return num(f.pts_ns) + "," + std::to_string(f.chain) + "," +
         from_name(f.pose.from) + "," + num(f.pose.fit) + "," +
         num(f.pose.decided_pts_ns) + "," + mat(f.pose.correction) + "," +
         mat(f.device_pose) + "\n";
}

std::string fit_line(const FitRecord& r) {
  const track::TrackFit& f = r.fit;
  std::string s = std::to_string(r.chain) + "," + num(r.serial) + "," +
                  num(r.first_pts_ns) + "," + num(r.last_pts_ns) + "," +
                  std::to_string(f.frames) + "," + std::to_string(f.passes) +
                  "," + state_name(r.judged.state) + "," +
                  num(r.prior.sigma_m) + "," + num(r.prior.sigma_rad) + "," +
                  mat(r.prior.correction) + "," + mat(f.correction) + "," +
                  mat(r.judged.correction) + "," + num(f.lambda_max);
  for (const float e : f.relative_eigenvalues) s += "," + num(e);
  s += "," + std::to_string(f.held) + "," + std::to_string(f.attempts) + "," +
       std::to_string(f.hits) + "," + std::to_string(f.inliers) + "," +
       std::to_string(f.in_front) + "," + std::to_string(f.behind) + "," +
       num(f.see_through_m) + "," + num(f.rms_m) + "," + num(f.origin.x) + "," +
       num(f.origin.y) + "," + num(f.origin.z) + "," + num(f.scale_m) + "," +
       one_line(r.refused) + "\n";
  return s;
}

std::string fit_header() {
  return "chain,serial,first_pts_ns,last_pts_ns,frames,passes,state,"
         "prior_sigma_m,prior_sigma_rad," +
         mat_header("prior") + "," + mat_header("fit") + "," +
         mat_header("judged") +
         ",lambda_max,rel_eig0,rel_eig1,rel_eig2,rel_eig3,rel_eig4,rel_eig5,"
         "held,attempts,hits,inliers,in_front,behind,see_through_m,rms_m,"
         "origin_x,origin_y,origin_z,scale_m,refused\n";
}

std::string fill_line(const PlacementRecord& p) {
  const char* kind = p.dropped ? "dropped"
                     : p.placement == track::Placement::Anchored
                         ? "anchored"
                         : "interpolated";
  return num(p.pts_ns) + "," + kind + "," +
         (p.dropped ? std::string() : std::to_string(p.driver)) + "\n";
}

std::string row_line(const RowSample& r) {
  return num(r.step) + "," + one_line(r.name) + "," + num(r.cpu_ms) + "," +
         num(r.gpu_ms) + "\n";
}

std::string tag_header() {
  return "first_pts_ns,last_pts_ns,stills,use,miss,rms_px,worst_px,range_m,"
         "off_normal_deg," +
         mat_header("tracking_to_world") + ",reason\n";
}

std::string tag_line(const TagHold& h) {
  return num(h.first_pts_ns) + "," + num(h.last_pts_ns) + "," +
         std::to_string(h.stills) + "," + to_string(h.use) + "," +
         one_line(h.miss) + "," + num(h.rms_px) + "," + num(h.worst_px) + "," +
         num(h.range_m) + "," + num(h.off_normal_deg) + "," +
         mat(h.tracking_to_world) + "," + one_line(h.reason) + "\n";
}

// --- Poses ------------------------------------------------------------------

using Mat3d = glm::dmat3;
using Vec3d = glm::dvec3;

Mat3d rot_of(const Mat4f& m) {
  Mat3d r(1.0);
  for (int c = 0; c < 3; ++c) {
    for (int k = 0; k < 3; ++k) r[c][k] = double(m[c][k]);
  }
  return r;
}
Vec3d centre_of(const Mat4f& m) {
  return Vec3d(double(m[3][0]), double(m[3][1]), double(m[3][2]));
}

double angle_deg(const Mat3d& r) {
  const Vec3d v(r[1][2] - r[2][1], r[2][0] - r[0][2], r[0][1] - r[1][0]);
  return std::atan2(0.5 * glm::length(v),
                    (r[0][0] + r[1][1] + r[2][2] - 1.0) * 0.5) *
         180.0 / kPi;
}

// The quaternion of a rotation, (x, y, z, w), w >= 0.
glm::dvec4 quaternion(const Mat3d& m) {
  // m[c][r]: element (r, c).
  const double m00 = m[0][0], m11 = m[1][1], m22 = m[2][2];
  const double tr = m00 + m11 + m22;
  glm::dvec4 q;
  if (tr > 0.0) {
    const double s = std::sqrt(tr + 1.0) * 2.0;
    q = {(m[1][2] - m[2][1]) / s, (m[2][0] - m[0][2]) / s,
         (m[0][1] - m[1][0]) / s, 0.25 * s};
  } else if (m00 > m11 && m00 > m22) {
    const double s = std::sqrt(1.0 + m00 - m11 - m22) * 2.0;
    q = {0.25 * s, (m[1][0] + m[0][1]) / s, (m[2][0] + m[0][2]) / s,
         (m[1][2] - m[2][1]) / s};
  } else if (m11 > m22) {
    const double s = std::sqrt(1.0 + m11 - m00 - m22) * 2.0;
    q = {(m[1][0] + m[0][1]) / s, 0.25 * s, (m[2][1] + m[1][2]) / s,
         (m[2][0] - m[0][2]) / s};
  } else {
    const double s = std::sqrt(1.0 + m22 - m00 - m11) * 2.0;
    q = {(m[2][0] + m[0][2]) / s, (m[2][1] + m[1][2]) / s, 0.25 * s,
         (m[0][1] - m[1][0]) / s};
  }
  if (q.w < 0.0) q = -q;
  return q / glm::length(q);
}

std::string tum_line(std::uint64_t pts, const Mat4f& pose) {
  const glm::dvec4 q = quaternion(rot_of(pose));
  const Vec3d c = centre_of(pose);
  char buf[256];
  std::snprintf(buf, sizeof(buf), "%.9f %.9g %.9g %.9g %.9g %.9g %.9g %.9g\n",
                double(pts) * 1e-9, c.x, c.y, c.z, q.x, q.y, q.z, q.w);
  return buf;
}

vkc::Status write_tum(const std::filesystem::path& path,
                      std::vector<std::pair<std::uint64_t, Mat4f>> poses) {
  std::stable_sort(
      poses.begin(), poses.end(),
      [](const auto& a, const auto& b) { return a.first < b.first; });
  std::string text;
  for (const auto& [pts, pose] : poses) text += tum_line(pts, pose);
  return write_text(path, text);
}

// The device's own pose of a frame the run's `arkit` posed at `arkit`:
// T * raw_from_sent(pts) * T^-1 * arkit.
Mat4f raw_pose(const RunSource& source, std::uint64_t pts, const Mat4f& arkit) {
  const DeviceStep* step = nullptr;
  for (const DeviceStep& d : source.device_steps) {
    if (d.pts_ns <= pts) step = &d;
  }
  if (step == nullptr) return arkit;
  return source.tracking_to_world * step->raw_from_sent *
         glm::inverse(source.tracking_to_world) * arkit;
}

Mat4f world_pose(const FrameRecord& f) {
  return f.pose.correction * f.device_pose;
}

}  // namespace

// --- TrackLog ---------------------------------------------------------------

struct TrackLog::State {
  std::string directory;      // as given
  std::filesystem::path dir;  // empty: no files
  TrackRun run;
  vkc::StageMetrics rows;
  std::uint64_t step = 0;
  std::ofstream frames, fits, fill, rowsf, tags;
  bool registered = false;  // by registered(), once
  bool closed = false;

  vkc::Status append(std::ofstream& file, const std::string& line,
                     const char* name) {
    if (dir.empty()) return {};
    file << line;
    if (!file) {
      return vkc::Status::io_error("TrackLog: cannot write " +
                                   (dir / name).string());
    }
    return {};
  }
};

TrackLog::TrackLog(std::unique_ptr<State> state) noexcept
    : state_(std::move(state)) {}
TrackLog::~TrackLog() = default;

vkc::Result<std::unique_ptr<TrackLog>> TrackLog::open(
    const std::string& directory, const track::Tracker& tracker,
    RunSource source) try {
  auto s = std::make_unique<State>();
  s->directory = directory;
  s->dir = directory;
  s->run.source = std::move(source);
  const track::TrackerConfig& config = tracker.config();
  s->run.hold_ns = config.hold_ns;
  const auto add = [&s](const std::string& name, bool driver,
                        const track::Method& m) {
    ChainInfo c;
    c.name = name;
    c.driver = driver;
    c.fit = m.fit;
    c.window_ns = m.window_ns;
    c.hop_ns = m.hop_ns;
    c.passes = m.fit ? m.icp.passes : 0;
    c.initial_correction = m.initial_correction;
    c.seed = m.seed;
    s->run.chains.push_back(std::move(c));
  };
  for (const track::DriverChain& d : config.drivers) {
    add(d.name, true, config.driver);
  }
  for (const track::Method& m : config.shadows) add(m.name, false, m);
  if (!s->dir.empty()) {
    std::error_code ec;
    std::filesystem::create_directories(s->dir, ec);
    if (ec) {
      return vkc::Status::io_error("TrackLog::open: cannot make " +
                                   s->dir.string() + ": " + ec.message());
    }
    VKC_TRY(write_text(s->dir / "run.csv", run_csv(s->run)));
    VKC_TRY(write_text(s->dir / "chains.csv", chains_csv(s->run)));
    VKC_TRY(write_text(s->dir / "steps.csv", steps_csv(s->run)));
    const std::pair<std::ofstream*, std::pair<const char*, std::string>>
        files[] = {{&s->frames,
                    {"frames.csv", "pts_ns,chain,from,fit,decided_pts_ns," +
                                       mat_header("correction") + "," +
                                       mat_header("device_pose") + "\n"}},
                   {&s->fits, {"fits.csv", fit_header()}},
                   {&s->fill, {"fill.csv", "pts_ns,placement,driver\n"}},
                   {&s->rowsf, {"rows.csv", "step,name,cpu_ms,gpu_ms\n"}},
                   {&s->tags, {"tags.csv", tag_header()}}};
    for (const auto& [file, what] : files) {
      file->open(s->dir / what.first, std::ios::binary | std::ios::trunc);
      *file << what.second;
      if (!*file) {
        return vkc::Status::io_error("TrackLog::open: cannot write " +
                                     (s->dir / what.first).string());
      }
    }
  }
  return std::unique_ptr<TrackLog>(new TrackLog(std::move(s)));
} catch (const std::bad_alloc&) {
  return vkc::Status::out_of_memory("TrackLog::open");
}

vkc::StageMetrics& TrackLog::rows() noexcept { return state_->rows; }

const TrackRun& TrackLog::run() const noexcept { return state_->run; }

const std::string& TrackLog::directory() const noexcept {
  return state_->directory;
}

vkc::Status TrackLog::write(const track::TrackStep& step) try {
  State& s = *state_;
  if (s.closed) return vkc::Status::invalid_argument("TrackLog: closed");
  for (const track::ChainPose& p : step.poses) {
    FrameRecord f;
    f.pts_ns = p.frame.timestamp_ns;
    f.chain = p.chain;
    f.pose = p.pose;
    f.device_pose = p.frame.depth_camera.cam_to_world;
    VKC_TRY(s.append(s.frames, frame_line(f), "frames.csv"));
    s.run.frames.push_back(f);
  }
  for (const track::ChainFit& c : step.fits) {
    FitRecord r;
    r.chain = c.chain;
    r.serial = c.serial;
    r.first_pts_ns = c.first_pts_ns;
    r.last_pts_ns = c.last_pts_ns;
    r.prior = c.prior;
    r.fit = c.fit;
    r.fit.directions = {};
    r.judged = c.judged;
    r.refused = one_line(c.refused);
    VKC_TRY(s.append(s.fits, fit_line(r), "fits.csv"));
    s.run.fits.push_back(std::move(r));
  }
  for (const track::Released& r : step.released) {
    PlacementRecord p;
    p.pts_ns = r.frame.timestamp_ns;
    p.placement = r.placement;
    p.driver = r.driver;
    VKC_TRY(s.append(s.fill, fill_line(p), "fill.csv"));
    s.run.placements.push_back(p);
  }
  for (const std::uint64_t pts : step.dropped_ns) {
    PlacementRecord p;
    p.pts_ns = pts;
    p.dropped = true;
    VKC_TRY(s.append(s.fill, fill_line(p), "fill.csv"));
    s.run.placements.push_back(p);
  }
  for (const vkc::StageRow& row : s.rows.rows()) {
    RowSample r;
    r.step = s.step;
    r.name = row.name;
    r.cpu_ms = row.cpu_ms;
    r.gpu_ms = row.device == vkc::DeviceTime::Measured ? row.gpu_ms : -1.0;
    VKC_TRY(s.append(s.rowsf, row_line(r), "rows.csv"));
    s.run.rows.push_back(std::move(r));
  }
  s.rows.clear();
  ++s.step;
  return {};
} catch (const std::bad_alloc&) {
  return vkc::Status::out_of_memory("TrackLog::write");
}

vkc::Status TrackLog::write(const TagHold& hold) try {
  State& s = *state_;
  if (s.closed) return vkc::Status::invalid_argument("TrackLog: closed");
  return s.append(s.tags, tag_line(hold), "tags.csv");
} catch (const std::bad_alloc&) {
  return vkc::Status::out_of_memory("TrackLog::write");
}

vkc::Status TrackLog::registered(std::string registration,
                                 const Mat4f& tracking_to_world) try {
  State& s = *state_;
  if (s.closed) return vkc::Status::invalid_argument("TrackLog: closed");
  if (s.registered) {
    return vkc::Status::invalid_argument(
        "TrackLog::registered: the run is registered already");
  }
  if (!s.run.frames.empty()) {
    return vkc::Status::invalid_argument(
        "TrackLog::registered: after a step with frames, whose poses the "
        "registration did not place");
  }
  s.run.source.registration = std::move(registration);
  s.run.source.tracking_to_world = tracking_to_world;
  s.registered = true;
  if (s.dir.empty()) return {};
  return write_text(s.dir / "run.csv", run_csv(s.run));
} catch (const std::bad_alloc&) {
  return vkc::Status::out_of_memory("TrackLog::registered");
}

std::pair<double, double> pose_gap(const Mat4f& a, const Mat4f& b) {
  const double mm = glm::length(centre_of(a) - centre_of(b)) * 1e3;
  const double deg = angle_deg(rot_of(a) * glm::transpose(rot_of(b)));
  return {mm, deg};
}

vkc::Status TrackLog::close() try {
  State& s = *state_;
  if (s.closed) return {};
  s.closed = true;
  if (s.dir.empty()) return {};
  for (std::ofstream* f : {&s.frames, &s.fits, &s.fill, &s.rowsf, &s.tags}) {
    f->close();
    if (!*f) return vkc::Status::io_error("TrackLog::close: a write failed");
  }
  for (std::size_t c = 0; c < s.run.chains.size(); ++c) {
    std::vector<std::pair<std::uint64_t, Mat4f>> poses;
    for (const FrameRecord& f : s.run.frames) {
      if (f.chain == c) poses.emplace_back(f.pts_ns, world_pose(f));
    }
    VKC_TRY(write_tum(s.dir / ("poses_" + s.run.chains[c].name + ".tum"),
                      std::move(poses)));
  }
  return {};
} catch (const std::bad_alloc&) {
  return vkc::Status::out_of_memory("TrackLog::close");
}

// --- Reading ----------------------------------------------------------------

namespace {

// A CSV file's lines after its header, each split at its commas; the last
// field takes the rest of the line when `rest_after` fields precede it.
struct Csv {
  std::string name;
  std::vector<std::vector<std::string>> rows;
};

vkc::Result<Csv> read_csv(const std::filesystem::path& path,
                          std::size_t rest_after = 0) {
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    return vkc::Status::not_found("read_track_log: no " + path.string());
  }
  Csv csv;
  csv.name = path.filename().string();
  std::string line;
  bool header = true;
  while (std::getline(in, line)) {
    if (header) {
      header = false;
      continue;
    }
    std::vector<std::string> fields;
    std::size_t begin = 0;
    for (;;) {
      if (rest_after != 0 && fields.size() == rest_after) {
        fields.push_back(line.substr(begin));
        break;
      }
      const std::size_t comma = line.find(',', begin);
      fields.push_back(line.substr(begin, comma - begin));
      if (comma == std::string::npos) break;
      begin = comma + 1;
    }
    csv.rows.push_back(std::move(fields));
  }
  return csv;
}

// Fields read in order from one row, any failure naming the file and line.
class Fields {
 public:
  Fields(const Csv& csv, std::size_t row) : csv_(csv), row_(row) {}
  bool ok() const { return ok_; }
  std::size_t left() const { return csv_.rows[row_].size() - next_; }
  std::string text() {
    if (next_ >= csv_.rows[row_].size()) {
      ok_ = false;
      return {};
    }
    return csv_.rows[row_][next_++];
  }
  std::uint64_t u64() {
    const std::string t = text();
    char* end = nullptr;
    errno = 0;
    const unsigned long long v = std::strtoull(t.c_str(), &end, 10);
    if (t.empty() || end != t.c_str() + t.size() || errno != 0) ok_ = false;
    return v;
  }
  double f64() {
    const std::string t = text();
    char* end = nullptr;
    const double v = std::strtod(t.c_str(), &end);
    if (t.empty() || end != t.c_str() + t.size()) ok_ = false;
    return v;
  }
  float f32() {
    const std::string t = text();
    char* end = nullptr;
    const float v = std::strtof(t.c_str(), &end);
    if (t.empty() || end != t.c_str() + t.size()) ok_ = false;
    return v;
  }
  Mat4f m4() {
    Mat4f m(1.0f);
    for (int c = 0; c < 4; ++c) {
      for (int r = 0; r < 4; ++r) m[c][r] = f32();
    }
    return m;
  }
  vkc::Status status() const {
    if (ok_) return {};
    return vkc::Status::invalid_argument("read_track_log: " + csv_.name +
                                         " line " + std::to_string(row_ + 2) +
                                         " does not parse");
  }

 private:
  const Csv& csv_;
  std::size_t row_;
  std::size_t next_ = 0;
  bool ok_ = true;
};

}  // namespace

vkc::Result<TrackRun> read_track_log(const std::string& directory) try {
  const std::filesystem::path dir(directory);
  TrackRun run;
  {
    VKC_ASSIGN(const Csv csv, read_csv(dir / "run.csv", 1));
    std::map<std::string, std::size_t> at;
    for (std::size_t i = 0; i < csv.rows.size(); ++i) {
      if (csv.rows[i].size() == 2) at[csv.rows[i][0]] = i;
    }
    const auto value = [&](const char* key) -> std::string {
      const auto it = at.find(key);
      return it == at.end() ? std::string() : csv.rows[it->second][1];
    };
    if (value("format") != kFormat || value("version") != "1") {
      return vkc::Status::invalid_argument("read_track_log: " + directory +
                                           " is not a " + kFormat +
                                           " version 1 log");
    }
    run.source.session = value("session");
    run.source.registration = value("registration");
    run.source.device = value("device");
    // Numbers: re-split the two numeric values.
    Csv nums;
    nums.name = csv.name;
    nums.rows.push_back({value("hold_ns")});
    std::vector<std::string> m;
    std::stringstream ss(value("tracking_to_world"));
    for (std::string part; std::getline(ss, part, ',');) m.push_back(part);
    nums.rows.push_back(m);
    Fields hold(nums, 0);
    run.hold_ns = hold.u64();
    Fields t(nums, 1);
    run.source.tracking_to_world = t.m4();
    VKC_TRY(hold.status());
    VKC_TRY(t.status());
  }
  {
    VKC_ASSIGN(const Csv csv, read_csv(dir / "chains.csv", 22));
    for (std::size_t i = 0; i < csv.rows.size(); ++i) {
      Fields f(csv, i);
      ChainInfo c;
      c.name = f.text();
      const std::string role = f.text();
      const std::optional<std::optional<track::FitKind>> kind =
          parse_kind(f.text());
      c.window_ns = f.u64();
      c.hop_ns = f.u64();
      c.passes = int(f.u64());
      c.initial_correction = f.m4();
      c.seed = f.text();
      VKC_TRY(f.status());
      if (!kind || (role != "driver" && role != "shadow")) {
        return vkc::Status::invalid_argument(
            "read_track_log: chains.csv line " + std::to_string(i + 2) +
            " does not parse");
      }
      c.driver = role == "driver";
      c.fit = *kind;
      run.chains.push_back(std::move(c));
    }
  }
  {
    VKC_ASSIGN(const Csv csv, read_csv(dir / "steps.csv"));
    for (std::size_t i = 0; i < csv.rows.size(); ++i) {
      Fields f(csv, i);
      DeviceStep d;
      d.pts_ns = f.u64();
      d.raw_from_sent = f.m4();
      VKC_TRY(f.status());
      run.source.device_steps.push_back(d);
    }
  }
  const auto chain_ok = [&run](std::size_t c) { return c < run.chains.size(); };
  {
    VKC_ASSIGN(const Csv csv, read_csv(dir / "frames.csv"));
    for (std::size_t i = 0; i < csv.rows.size(); ++i) {
      Fields f(csv, i);
      FrameRecord r;
      r.pts_ns = f.u64();
      r.chain = std::size_t(f.u64());
      const std::optional<track::PoseFrom> from = parse_from(f.text());
      r.pose.fit = f.u64();
      r.pose.decided_pts_ns = f.u64();
      r.pose.correction = f.m4();
      r.device_pose = f.m4();
      if (!from || !chain_ok(r.chain)) {
        return vkc::Status::invalid_argument(
            "read_track_log: frames.csv line " + std::to_string(i + 2) +
            " does not parse");
      }
      VKC_TRY(f.status());
      r.pose.from = *from;
      run.frames.push_back(r);
    }
  }
  {
    // 76 fields before `refused`, the rest of the line.
    VKC_ASSIGN(const Csv csv, read_csv(dir / "fits.csv", 76));
    for (std::size_t i = 0; i < csv.rows.size(); ++i) {
      Fields f(csv, i);
      FitRecord r;
      r.chain = std::size_t(f.u64());
      r.serial = f.u64();
      r.first_pts_ns = f.u64();
      r.last_pts_ns = f.u64();
      r.fit.frames = std::uint32_t(f.u64());
      r.fit.passes = std::uint8_t(f.u64());
      const std::optional<track::TrackState> state = parse_state(f.text());
      r.prior.sigma_m = f.f32();
      r.prior.sigma_rad = f.f32();
      r.prior.correction = f.m4();
      r.fit.correction = f.m4();
      r.judged.correction = f.m4();
      r.fit.lambda_max = f.f32();
      for (float& e : r.fit.relative_eigenvalues) e = f.f32();
      r.fit.held = std::uint8_t(f.u64());
      r.fit.attempts = std::uint32_t(f.u64());
      r.fit.hits = std::uint32_t(f.u64());
      r.fit.inliers = std::uint32_t(f.u64());
      r.fit.in_front = std::uint32_t(f.u64());
      r.fit.behind = std::uint32_t(f.u64());
      r.fit.see_through_m = f.f32();
      r.fit.rms_m = f.f32();
      r.fit.origin.x = f.f32();
      r.fit.origin.y = f.f32();
      r.fit.origin.z = f.f32();
      r.fit.scale_m = f.f32();
      r.refused = f.text();
      if (!state || !chain_ok(r.chain)) {
        return vkc::Status::invalid_argument("read_track_log: fits.csv line " +
                                             std::to_string(i + 2) +
                                             " does not parse");
      }
      VKC_TRY(f.status());
      r.judged.state = *state;
      run.fits.push_back(std::move(r));
    }
  }
  {
    VKC_ASSIGN(const Csv csv, read_csv(dir / "fill.csv"));
    for (std::size_t i = 0; i < csv.rows.size(); ++i) {
      Fields f(csv, i);
      PlacementRecord p;
      p.pts_ns = f.u64();
      const std::string kind = f.text();
      if (kind == "dropped") {
        p.dropped = true;
      } else if (kind == "anchored" || kind == "interpolated") {
        p.placement = kind == "anchored" ? track::Placement::Anchored
                                         : track::Placement::Interpolated;
        p.driver = std::size_t(f.u64());
      } else {
        return vkc::Status::invalid_argument("read_track_log: fill.csv line " +
                                             std::to_string(i + 2) +
                                             " does not parse");
      }
      VKC_TRY(f.status());
      run.placements.push_back(p);
    }
  }
  {
    VKC_ASSIGN(const Csv csv, read_csv(dir / "rows.csv"));
    for (std::size_t i = 0; i < csv.rows.size(); ++i) {
      Fields f(csv, i);
      RowSample r;
      r.step = f.u64();
      r.name = f.text();
      r.cpu_ms = f.f64();
      r.gpu_ms = f.f64();
      VKC_TRY(f.status());
      run.rows.push_back(std::move(r));
    }
  }
  return run;
} catch (const std::bad_alloc&) {
  return vkc::Status::out_of_memory("read_track_log");
}

// --- The summary ------------------------------------------------------------

namespace {

// Nearest-rank percentile of `v` (sorted here); NaN for none.
double pct(std::vector<double> v, double p) {
  if (v.empty()) return std::nan("");
  std::sort(v.begin(), v.end());
  const auto k = std::size_t(std::ceil(p / 100.0 * double(v.size())));
  return v[std::min(v.size() - 1, k == 0 ? 0 : k - 1)];
}

double max_of(const std::vector<double>& v) {
  return v.empty() ? std::nan("") : *std::max_element(v.begin(), v.end());
}

std::string cell(double v, const char* fmt = "%.1f") {
  if (std::isnan(v)) return "-";
  char buf[32];
  std::snprintf(buf, sizeof(buf), fmt, v);
  return buf;
}

struct Distances {
  std::vector<double> mm, deg;
};

struct ChainRow {
  std::string name;
  std::size_t decided = 0;
  std::size_t by_from[4] = {0, 0, 0, 0};
  std::vector<double> latency_ms;
  Distances from_arkit;
  double jitter_mm = std::nan("");
  double jitter_deg = std::nan("");
  std::size_t fits = 0;
  std::size_t by_state[4] = {0, 0, 0, 0};
  std::size_t refused = 0;
  bool pose_only = false;
};

}  // namespace

vkc::Result<std::string> summarize(const TrackRun& run,
                                   const std::string& directory) try {
  // Each chain's decisions by frame time.
  std::vector<std::map<std::uint64_t, const FrameRecord*>> by_chain(
      run.chains.size());
  for (const FrameRecord& f : run.frames) {
    if (f.chain < by_chain.size()) by_chain[f.chain][f.pts_ns] = &f;
  }
  std::optional<std::size_t> arkit;
  for (std::size_t c = 0; c < run.chains.size(); ++c) {
    if (run.chains[c].name == "arkit" && !run.chains[c].fit) arkit = c;
  }
  std::vector<ChainRow> rows;
  for (std::size_t c = 0; c < run.chains.size(); ++c) {
    ChainRow row;
    row.name = run.chains[c].name;
    std::vector<double> step_mm, step_deg;
    const FrameRecord* last_anchored = nullptr;
    for (const auto& [pts, f] : by_chain[c]) {
      ++row.decided;
      ++row.by_from[std::size_t(f->pose.from)];
      row.latency_ms.push_back((double(f->pose.decided_pts_ns) - double(pts)) *
                               1e-6);
      if (arkit && c != *arkit) {
        const auto a = by_chain[*arkit].find(pts);
        if (a != by_chain[*arkit].end()) {
          const auto [mm, deg] =
              pose_gap(world_pose(*f), world_pose(*a->second));
          row.from_arkit.mm.push_back(mm);
          row.from_arkit.deg.push_back(deg);
        }
      }
      if (f->pose.from == track::PoseFrom::Anchored) {
        if (last_anchored != nullptr) {
          // The change of the correction, at this frame's camera.
          const Mat4f now = world_pose(*f);
          const Mat4f then = last_anchored->pose.correction * f->device_pose;
          const auto [mm, deg] = pose_gap(now, then);
          step_mm.push_back(mm * mm);
          step_deg.push_back(deg * deg);
        }
        last_anchored = f;
      }
    }
    if (!step_mm.empty()) {
      double sm = 0.0, sd = 0.0;
      for (std::size_t i = 0; i < step_mm.size(); ++i) {
        sm += step_mm[i];
        sd += step_deg[i];
      }
      row.jitter_mm = std::sqrt(sm / double(step_mm.size()));
      row.jitter_deg = std::sqrt(sd / double(step_deg.size()));
    }
    for (const FitRecord& f : run.fits) {
      if (f.chain != c) continue;
      if (!f.refused.empty()) {
        ++row.refused;
        continue;
      }
      ++row.fits;
      ++row.by_state[std::size_t(f.judged.state)];
    }
    rows.push_back(std::move(row));
  }
  // The device's own poses, rebuilt from arkit's and the removed steps.
  std::vector<std::pair<std::uint64_t, Mat4f>> raw_poses;
  if (arkit && !run.source.device_steps.empty()) {
    ChainRow row;
    row.name = "arkit_raw";
    row.pose_only = true;
    for (const auto& [pts, f] : by_chain[*arkit]) {
      const Mat4f a = world_pose(*f);
      const Mat4f raw = raw_pose(run.source, pts, a);
      raw_poses.emplace_back(pts, raw);
      const auto [mm, deg] = pose_gap(raw, a);
      row.from_arkit.mm.push_back(mm);
      row.from_arkit.deg.push_back(deg);
      ++row.decided;
    }
    rows.push_back(std::move(row));
  }

  std::string table;
  char buf[512];
  std::snprintf(buf, sizeof(buf),
                "tracking: %zu chains, hold %.1f s, %zu frames decided, %zu "
                "fits\n",
                run.chains.size(), double(run.hold_ns) * 1e-9,
                run.frames.size(), run.fits.size());
  table += buf;
  std::snprintf(buf, sizeof(buf),
                "  %-12s %7s %6s %6s %6s %6s %6s %9s %17s %17s %13s %20s\n",
                "chain", "decided", "device", "anch", "interp", "coast",
                "lock%", "lat p50/95", "from arkit mm", "from arkit deg",
                "jitter", "fits a/p/c/r/refused");
  table += buf;
  std::string csv =
      "chain,decided,device,anchored,interpolated,coasting,lock_share,"
      "latency_p50_ms,latency_p95_ms,dist_p50_mm,dist_p95_mm,dist_max_mm,"
      "dist_p50_deg,dist_p95_deg,dist_max_deg,jitter_mm,jitter_deg,fits,"
      "fits_anchored,fits_partial,fits_coasting,fits_rejected,fits_refused\n";
  for (const ChainRow& r : rows) {
    const double lock = r.decided == 0 || r.pose_only
                            ? std::nan("")
                            : 100.0 * double(r.by_from[1]) / double(r.decided);
    const double l50 = r.pose_only ? std::nan("") : pct(r.latency_ms, 50);
    const double l95 = r.pose_only ? std::nan("") : pct(r.latency_ms, 95);
    const double d50 = pct(r.from_arkit.mm, 50), d95 = pct(r.from_arkit.mm, 95);
    const double dmax = max_of(r.from_arkit.mm);
    const double a50 = pct(r.from_arkit.deg, 50),
                 a95 = pct(r.from_arkit.deg, 95);
    const double amax = max_of(r.from_arkit.deg);
    const std::string counts = r.pose_only
                                   ? std::string("-")
                                   : std::to_string(r.by_state[0]) + "/" +
                                         std::to_string(r.by_state[1]) + "/" +
                                         std::to_string(r.by_state[2]) + "/" +
                                         std::to_string(r.by_state[3]) + "/" +
                                         std::to_string(r.refused);
    const auto from = [&r](std::size_t i) {
      return r.pose_only ? std::string("-") : std::to_string(r.by_from[i]);
    };
    std::snprintf(
        buf, sizeof(buf),
        "  %-12s %7zu %6s %6s %6s %6s %6s %9s %17s %17s %13s %20s\n",
        r.name.c_str(), r.decided, from(0).c_str(), from(1).c_str(),
        from(2).c_str(), from(3).c_str(), cell(lock).c_str(),
        (cell(l50, "%.0f") + "/" + cell(l95, "%.0f")).c_str(),
        (cell(d50) + "/" + cell(d95) + "/" + cell(dmax)).c_str(),
        (cell(a50, "%.2f") + "/" + cell(a95, "%.2f") + "/" + cell(amax, "%.2f"))
            .c_str(),
        (cell(r.jitter_mm) + "/" + cell(r.jitter_deg, "%.2f")).c_str(),
        counts.c_str());
    table += buf;
    const auto v = [](double x) {
      return std::isnan(x) ? std::string() : num(x);
    };
    const auto n = [&r](std::size_t x) {
      return r.pose_only ? std::string() : std::to_string(x);
    };
    csv += r.name + "," + std::to_string(r.decided) + "," + n(r.by_from[0]) +
           "," + n(r.by_from[1]) + "," + n(r.by_from[2]) + "," +
           n(r.by_from[3]) + "," + v(lock / 100.0) + "," + v(l50) + "," +
           v(l95) + "," + v(d50) + "," + v(d95) + "," + v(dmax) + "," + v(a50) +
           "," + v(a95) + "," + v(amax) + "," + v(r.jitter_mm) + "," +
           v(r.jitter_deg) + "," + n(r.fits) + "," + n(r.by_state[0]) + "," +
           n(r.by_state[1]) + "," + n(r.by_state[2]) + "," + n(r.by_state[3]) +
           "," + n(r.refused) + "\n";
  }
  // The driver's fill: placements by kind and chain, and the drops.
  std::map<std::pair<std::size_t, int>, std::size_t> placed;
  std::size_t drops = 0;
  for (const PlacementRecord& p : run.placements) {
    if (p.dropped) {
      ++drops;
    } else {
      ++placed[{p.driver, int(p.placement)}];
    }
  }
  table += "  fill:";
  for (std::size_t c = 0; c < run.chains.size(); ++c) {
    if (!run.chains[c].driver) continue;
    std::snprintf(buf, sizeof(buf), " %s anchored %zu, interpolated %zu;",
                  run.chains[c].name.c_str(), placed[{c, 0}], placed[{c, 1}]);
    table += buf;
  }
  std::snprintf(buf, sizeof(buf), " dropped %zu\n", drops);
  table += buf;
  // Every stage row a step: p50 / p95 of its host and device halves.
  std::vector<std::string> names;
  std::map<std::string, std::pair<std::vector<double>, std::vector<double>>>
      samples;
  for (const RowSample& r : run.rows) {
    auto [it, fresh] = samples.try_emplace(r.name);
    if (fresh) names.push_back(r.name);
    it->second.first.push_back(r.cpu_ms);
    if (r.gpu_ms >= 0.0) it->second.second.push_back(r.gpu_ms);
  }
  if (!names.empty()) {
    std::snprintf(buf, sizeof(buf), "  %-28s %6s %17s %17s\n", "row (a step)",
                  "steps", "host ms p50/p95", "device ms p50/95");
    table += buf;
  }
  for (const std::string& name : names) {
    const auto& [cpu, gpu] = samples[name];
    std::snprintf(
        buf, sizeof(buf), "  %-28s %6zu %17s %17s\n", name.c_str(), cpu.size(),
        (cell(pct(cpu, 50), "%.3f") + "/" + cell(pct(cpu, 95), "%.3f")).c_str(),
        (cell(pct(gpu, 50), "%.3f") + "/" + cell(pct(gpu, 95), "%.3f"))
            .c_str());
    table += buf;
  }
  if (!directory.empty()) {
    const std::filesystem::path dir(directory);
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    if (ec) {
      return vkc::Status::io_error("summarize: cannot make " + dir.string() +
                                   ": " + ec.message());
    }
    VKC_TRY(write_text(dir / "summary.csv", csv));
    if (!raw_poses.empty()) {
      VKC_TRY(write_tum(dir / "poses_arkit_raw.tum", std::move(raw_poses)));
    }
  }
  return table;
} catch (const std::bad_alloc&) {
  return vkc::Status::out_of_memory("summarize");
}

}  // namespace vr_example
