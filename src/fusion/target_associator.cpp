#include "br_perception/fusion/target_associator.hpp"

#include <algorithm>
#include <cmath>

namespace br_perception {
namespace fusion {

namespace {

/// 与 `Track.msg` 的 SRC_* 一一对应（**别在这里另起一套枚举**）
constexpr std::uint8_t kSrcNone   = 0;
constexpr std::uint8_t kSrcLidar  = 1;
constexpr std::uint8_t kSrcCamera = 2;

/// 贪心候选：{代价, 观测下标, 追踪下标}
struct Candidate
{
  double cost;
  int obs;
  int track;
};

/// 按代价升序（代价相同时用下标兜底 —— 保证**同一份输入永远得到同一份输出**）
bool by_cost(const Candidate& a, const Candidate& b)
{
  if (a.cost != b.cost) return a.cost < b.cost;
  if (a.obs  != b.obs)  return a.obs  < b.obs;
  return a.track < b.track;
}

}  // namespace

// ═══════════════════════════════════════════════════════════════════════════
// 基础量
// ═══════════════════════════════════════════════════════════════════════════
double TargetAssociator::distance3d(double ax, double ay, double az,
                                    double bx, double by, double bz)
{
  const double dx = ax - bx, dy = ay - by, dz = az - bz;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool TargetAssociator::in_region(double x, double y,
                                 const KnownRegion& region,
                                 double default_radius_m)
{
  // ⚠️ **只看水平 xy**（理由见头文件：雷达对 z 的估计最差，带上 z 会让"明明就在那个位上"
  //    的观测频繁认领失败）。
  const double r = (region.radius_m > 0.0) ? region.radius_m : default_radius_m;
  const double dx = x - region.x;
  const double dy = y - region.y;
  // ⚠️ 边界是**非严格**（<=）：恰好落在半径上算"在里面"。
  //    与"严格 <"的选择本身不关键，但要**写死并测**，否则换个实现就悄悄改语义。
  return (dx * dx + dy * dy) <= (r * r);
}

const char* TargetAssociator::source_name(std::uint8_t source)
{
  switch (source) {
    case kSrcLidar:  return "lidar";
    case kSrcCamera: return "camera";
    case kSrcNone:   return "none";
    default:         return "unknown";
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// 关联
// ═══════════════════════════════════════════════════════════════════════════
std::vector<TargetAssociator::Match> TargetAssociator::associate(
    const std::vector<Observation>& obs,
    const std::vector<Track>& tracks,
    const std::vector<KnownRegion>& known,
    const Params& params)
{
  // ⚠️ **返回与 obs 等长同序**（未匹配的 track_index = -1）——
  //    调用方据此决定"哪条观测要建新候选"，见头文件的 ⚠️。
  std::vector<Match> result(obs.size());
  for (std::size_t i = 0; i < obs.size(); ++i) {
    result[i].obs_index = static_cast<int>(i);
    result[i].track_index = -1;
  }
  if (obs.empty() || tracks.empty()) return result;

  std::vector<bool> obs_taken(obs.size(), false);
  std::vector<bool> track_taken(tracks.size(), false);

  // 追踪 id → 下标（`.id` 是 `Track.msg` 里的身份，关联时按它找）
  auto find_track_by_id = [&tracks](std::uint32_t id) -> int {
    for (std::size_t j = 0; j < tracks.size(); ++j) {
      if (tracks[j].id == id) return static_cast<int>(j);
    }
    return -1;
  };

  // ── ① 落区认领（**优先**）──────────────────────────────────────────────
  //    观测落在某个**已知位置**的区域内 ⇒ 直接认领那个目标。
  //    理由：已知位置是**场地事实**（常量），而其它 track 的位置是**滤波估计**（会被带偏）。
  {
    std::vector<Candidate> cands;
    for (std::size_t i = 0; i < obs.size(); ++i) {
      // ⚠️⚠️ **类型门控**（2026-09-25 实测抓出）：落区认领原先**只看位置不看类型**
      //    ⇒ 一个敌方机器人只要靠近建筑位（实测 0.283m < 0.35m）就会被那个
      //    **静态建筑位**认领走 ⇒ 它不再是移动目标 ⇒ **BR_RESET 也清不掉它**。
      //    ⇒ 只有"可能被放在已知位置上"的类型才允许被认领。
      if (std::find(params.region_claim_types.begin(),
                    params.region_claim_types.end(), obs[i].type)
          == params.region_claim_types.end()) {
        continue;
      }
      for (const auto& reg : known) {
        if (!in_region(obs[i].x, obs[i].y, reg, params.default_region_m)) continue;
        const int j = find_track_by_id(reg.track_id);
        // ⚠️ 区域指向的目标**当前不在追踪列表里** ⇒ 跳过（不凭空造 track；
        //    建新候选是调用方的事）。
        if (j < 0) continue;
        cands.push_back({distance3d(obs[i].x, obs[i].y, obs[i].z,
                                    reg.x, reg.y, reg.z),
                         static_cast<int>(i), j});
      }
    }
    std::sort(cands.begin(), cands.end(), by_cost);
    for (const auto& c : cands) {
      if (obs_taken[c.obs] || track_taken[c.track]) continue;
      result[c.obs].track_index = c.track;
      result[c.obs].cost = c.cost;
      result[c.obs].by_region = true;      // ⚠️ 标记出来，便于诊断区分两级
      obs_taken[c.obs] = true;
      track_taken[c.track] = true;
    }
  }

  // ── ② 就近指派（**剩下的**按三维距离贪心，带 gate）────────────────────
  {
    std::vector<Candidate> cands;
    for (std::size_t i = 0; i < obs.size(); ++i) {
      if (obs_taken[i]) continue;
      for (std::size_t j = 0; j < tracks.size(); ++j) {
        if (track_taken[j]) continue;
        // ⚠️⚠️ **已知位置的目标不参与就近指派**（2026-09-25 实测抓出）：
        //    它在①里只接受"落在自己区域内"的观测；参与②的话，一个**会动的**目标
        //    只要离它够近（实测 0.354m < 0.6m 门限）就会被它抢走
        //    ⇒ 那个目标不再是移动目标 ⇒ **`BR_RESET` 清不掉它**。
        if (tracks[j].is_known_position) continue;
        // 「v1.3」持握目标**不参与雷达关联**：雷达点云与夹爪粘连、或方块不在建筑位上方
        // 被 building_spot_analyzer 忽略 ⇒ 拿雷达去"纠正"它只会把它拽错。
        if (params.hold_rejects_lidar && tracks[j].is_held &&
            obs[i].source == kSrcLidar) {
          continue;
        }
        const double d = distance3d(obs[i].x, obs[i].y, obs[i].z,
                                    tracks[j].x, tracks[j].y, tracks[j].z);
        // ⚠️ 门限是**非严格**（<=）。超出就**不关联** —— 该观测应去建新候选，
        //    而不是硬塞给最近的 track（那会把噪声塞进已有目标）。
        if (d <= params.gate_m) {
          cands.push_back({d, static_cast<int>(i), static_cast<int>(j)});
        }
      }
    }
    std::sort(cands.begin(), cands.end(), by_cost);
    for (const auto& c : cands) {
      if (obs_taken[c.obs] || track_taken[c.track]) continue;
      result[c.obs].track_index = c.track;
      result[c.obs].cost = c.cost;
      result[c.obs].by_region = false;
      obs_taken[c.obs] = true;
      track_taken[c.track] = true;
    }
  }

  return result;
}

}  // namespace fusion
}  // namespace br_perception
