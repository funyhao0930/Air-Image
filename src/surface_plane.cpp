#include "aerial_touch/surface_plane.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace aerial_touch {
namespace {

constexpr float kMinimumVectorLength = 0.001F;
constexpr std::size_t kMaximumPlaneHypotheses = 512U;

bool finite(const Vec3 value) {
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

Vec3 subtract(const Vec3 left, const Vec3 right) {
    return { left.x - right.x, left.y - right.y, left.z - right.z };
}

Vec3 add(const Vec3 left, const Vec3 right) {
    return { left.x + right.x, left.y + right.y, left.z + right.z };
}

Vec3 scale(const Vec3 value, const float factor) {
    return { value.x * factor, value.y * factor, value.z * factor };
}

float dot(const Vec3 left, const Vec3 right) {
    return left.x * right.x + left.y * right.y + left.z * right.z;
}

Vec3 cross(const Vec3 left, const Vec3 right) {
    return { left.y * right.z - left.z * right.y, left.z * right.x - left.x * right.z,
             left.x * right.y - left.y * right.x };
}

float length(const Vec3 value) {
    return std::sqrt(dot(value, value));
}

std::optional<Vec3> normalized(const Vec3 value) {
    const float magnitude = length(value);
    if(!std::isfinite(magnitude) || magnitude < kMinimumVectorLength) {
        return std::nullopt;
    }
    return scale(value, 1.0F / magnitude);
}

std::optional<Vec3> normal_from_points(const Vec3 first, const Vec3 second, const Vec3 third) {
    return normalized(cross(subtract(second, first), subtract(third, first)));
}

Vec3 centroid(const std::vector<Vec3>& points, const std::vector<std::size_t>& indices) {
    Vec3 total{};
    for(const std::size_t index : indices) {
        total = add(total, points[index]);
    }
    return scale(total, 1.0F / static_cast<float>(indices.size()));
}

std::optional<Vec3> smallest_covariance_axis(const std::vector<Vec3>& points,
                                              const std::vector<std::size_t>& indices,
                                              const Vec3 center) {
    double matrix[3][3]{};
    for(const std::size_t index : indices) {
        const Vec3 offset = subtract(points[index], center);
        const double values[3]{ offset.x, offset.y, offset.z };
        for(int row = 0; row < 3; ++row) {
            for(int column = 0; column < 3; ++column) {
                matrix[row][column] += values[row] * values[column];
            }
        }
    }

    double eigenvectors[3][3]{ { 1.0, 0.0, 0.0 }, { 0.0, 1.0, 0.0 }, { 0.0, 0.0, 1.0 } };
    for(int iteration = 0; iteration < 24; ++iteration) {
        int first = 0;
        int second = 1;
        double largest = std::fabs(matrix[first][second]);
        for(int row = 0; row < 3; ++row) {
            for(int column = row + 1; column < 3; ++column) {
                const double candidate = std::fabs(matrix[row][column]);
                if(candidate > largest) {
                    first = row;
                    second = column;
                    largest = candidate;
                }
            }
        }
        if(largest < 1.0e-9) {
            break;
        }

        const double angle = 0.5 * std::atan2(2.0 * matrix[first][second],
                                               matrix[second][second] - matrix[first][first]);
        const double cosine = std::cos(angle);
        const double sine = std::sin(angle);
        for(int index = 0; index < 3; ++index) {
            const double left = matrix[index][first];
            const double right = matrix[index][second];
            matrix[index][first] = cosine * left - sine * right;
            matrix[index][second] = sine * left + cosine * right;
        }
        for(int index = 0; index < 3; ++index) {
            const double top = matrix[first][index];
            const double bottom = matrix[second][index];
            matrix[first][index] = cosine * top - sine * bottom;
            matrix[second][index] = sine * top + cosine * bottom;
        }
        for(int index = 0; index < 3; ++index) {
            const double left = eigenvectors[index][first];
            const double right = eigenvectors[index][second];
            eigenvectors[index][first] = cosine * left - sine * right;
            eigenvectors[index][second] = sine * left + cosine * right;
        }
    }

    int smallest = 0;
    if(matrix[1][1] < matrix[smallest][smallest]) {
        smallest = 1;
    }
    if(matrix[2][2] < matrix[smallest][smallest]) {
        smallest = 2;
    }
    return normalized({ static_cast<float>(eigenvectors[0][smallest]), static_cast<float>(eigenvectors[1][smallest]),
                        static_cast<float>(eigenvectors[2][smallest]) });
}

std::vector<std::size_t> inlier_indices(const std::vector<Vec3>& points,
                                         const Vec3 origin,
                                         const Vec3 normal,
                                         const float threshold_mm) {
    std::vector<std::size_t> indices;
    indices.reserve(points.size());
    for(std::size_t index = 0; index < points.size(); ++index) {
        if(std::fabs(dot(subtract(points[index], origin), normal)) <= threshold_mm) {
            indices.push_back(index);
        }
    }
    return indices;
}

float rms_residual(const std::vector<Vec3>& points,
                   const std::vector<std::size_t>& indices,
                   const Vec3 origin,
                   const Vec3 normal) {
    float squared_total = 0.0F;
    for(const std::size_t index : indices) {
        const float residual = dot(subtract(points[index], origin), normal);
        squared_total += residual * residual;
    }
    return std::sqrt(squared_total / static_cast<float>(indices.size()));
}

bool valid_config(const SurfacePlaneFitConfig config) {
    return config.minimum_samples >= 3U && std::isfinite(config.minimum_inlier_ratio)
           && config.minimum_inlier_ratio > 0.0F && config.minimum_inlier_ratio <= 1.0F
           && std::isfinite(config.inlier_threshold_mm) && config.inlier_threshold_mm > 0.0F
           && std::isfinite(config.maximum_rms_residual_mm) && config.maximum_rms_residual_mm > 0.0F
           && std::isfinite(config.minimum_extent_mm) && config.minimum_extent_mm >= 0.0F;
}

// 2 sigma of the inliers along the narrower of the two in-plane principal directions. A sample set
// swept along a ribbon returns roughly its ribbon width here, whatever its residual looks like.
float minor_in_plane_extent(const std::vector<Vec3>& points,
                            const std::vector<std::size_t>& indices,
                            const Vec3 origin,
                            const Vec3 normal) {
    if(indices.size() < 2U) {
        return 0.0F;
    }

    // Any orthonormal pair spanning the plane will do; the eigen-decomposition below is basis free.
    const Vec3 seed = std::fabs(normal.x) < 0.9F ? Vec3{ 1.0F, 0.0F, 0.0F } : Vec3{ 0.0F, 1.0F, 0.0F };
    const auto axis_u = normalized(subtract(seed, scale(normal, dot(seed, normal))));
    if(!axis_u.has_value()) {
        return 0.0F;
    }
    const auto axis_v = normalized(cross(normal, *axis_u));
    if(!axis_v.has_value()) {
        return 0.0F;
    }

    double sum_u = 0.0;
    double sum_v = 0.0;
    for(const std::size_t index : indices) {
        const Vec3 offset = subtract(points[index], origin);
        sum_u += dot(offset, *axis_u);
        sum_v += dot(offset, *axis_v);
    }
    const double count = static_cast<double>(indices.size());
    const double mean_u = sum_u / count;
    const double mean_v = sum_v / count;

    double cov_uu = 0.0;
    double cov_uv = 0.0;
    double cov_vv = 0.0;
    for(const std::size_t index : indices) {
        const Vec3 offset = subtract(points[index], origin);
        const double u = dot(offset, *axis_u) - mean_u;
        const double v = dot(offset, *axis_v) - mean_v;
        cov_uu += u * u;
        cov_uv += u * v;
        cov_vv += v * v;
    }
    cov_uu /= count;
    cov_uv /= count;
    cov_vv /= count;

    // Smaller eigenvalue of the symmetric 2x2 covariance.
    const double trace = cov_uu + cov_vv;
    const double gap = std::sqrt(std::max(0.0, (cov_uu - cov_vv) * (cov_uu - cov_vv) + 4.0 * cov_uv * cov_uv));
    const double smallest = std::max(0.0, 0.5 * (trace - gap));
    return 2.0F * static_cast<float>(std::sqrt(smallest));
}

}  // namespace

SurfacePlane::SurfacePlane(const Vec3 origin, const Vec3 normal) : origin_(origin), normal_(normal) {}

std::optional<SurfacePlane> SurfacePlane::fit(const std::vector<Vec3>& samples,
                                              const SurfacePlaneFitConfig config,
                                              SurfacePlaneFitQuality* quality) {
    if(quality != nullptr) {
        *quality = {};
    }
    if(!valid_config(config)) {
        return std::nullopt;
    }

    std::vector<Vec3> valid_samples;
    valid_samples.reserve(samples.size());
    for(const Vec3 sample : samples) {
        if(finite(sample)) {
            valid_samples.push_back(sample);
        }
    }
    if(quality != nullptr) {
        quality->total_samples = valid_samples.size();
    }
    if(valid_samples.size() < config.minimum_samples) {
        return std::nullopt;
    }

    std::vector<std::size_t> best_inliers;
    Vec3 best_origin{};
    Vec3 best_normal{};
    float best_error = std::numeric_limits<float>::infinity();
    const std::size_t sample_count = valid_samples.size();
    const std::size_t hypothesis_count = std::min(kMaximumPlaneHypotheses, sample_count * 8U);
    for(std::size_t hypothesis = 0; hypothesis < hypothesis_count; ++hypothesis) {
        const std::size_t first = hypothesis % sample_count;
        const std::size_t second = (hypothesis * 37U + 11U) % sample_count;
        const std::size_t third = (hypothesis * 101U + 23U) % sample_count;
        if(first == second || first == third || second == third) {
            continue;
        }
        const auto normal = normal_from_points(valid_samples[first], valid_samples[second], valid_samples[third]);
        if(!normal.has_value()) {
            continue;
        }
        const auto inliers = inlier_indices(valid_samples, valid_samples[first], *normal, config.inlier_threshold_mm);
        if(inliers.empty()) {
            continue;
        }
        const float error = rms_residual(valid_samples, inliers, valid_samples[first], *normal);
        if(inliers.size() > best_inliers.size()
           || (inliers.size() == best_inliers.size() && error < best_error)) {
            best_inliers = inliers;
            best_origin = valid_samples[first];
            best_normal = *normal;
            best_error = error;
        }
    }

    const std::size_t required_inliers = static_cast<std::size_t>(std::ceil(
        config.minimum_inlier_ratio * static_cast<float>(valid_samples.size())));
    if(best_inliers.size() < required_inliers) {
        return std::nullopt;
    }

    Vec3 refined_origin = centroid(valid_samples, best_inliers);
    const auto refined_normal = smallest_covariance_axis(valid_samples, best_inliers, refined_origin);
    if(!refined_normal.has_value()) {
        return std::nullopt;
    }
    Vec3 normal = *refined_normal;
    if(dot(normal, scale(refined_origin, -1.0F)) < 0.0F) {
        normal = scale(normal, -1.0F);
    }
    const auto refined_inliers = inlier_indices(valid_samples, refined_origin, normal, config.inlier_threshold_mm);
    if(refined_inliers.size() < required_inliers) {
        return std::nullopt;
    }
    refined_origin = centroid(valid_samples, refined_inliers);
    const float residual = rms_residual(valid_samples, refined_inliers, refined_origin, normal);
    const float extent = minor_in_plane_extent(valid_samples, refined_inliers, refined_origin, normal);
    if(quality != nullptr) {
        quality->inlier_samples = refined_inliers.size();
        quality->rms_residual_mm = residual;
        quality->minor_extent_mm = extent;
    }
    if(!std::isfinite(residual) || residual > config.maximum_rms_residual_mm) {
        return std::nullopt;
    }
    if(!std::isfinite(extent) || extent < config.minimum_extent_mm) {
        // Well-fitted but not actually determined: the normal is free to rotate about the long axis.
        return std::nullopt;
    }
    return SurfacePlane(refined_origin, normal);
}

std::optional<Vec3> SurfacePlane::project_to_surface(const Vec3 point) const {
    if(!finite(point)) {
        return std::nullopt;
    }
    return subtract(point, scale(normal_, signed_distance(point)));
}

std::optional<Vec3> SurfacePlane::intersect_ray(const Vec3 origin, const Vec3 direction) const {
    const auto normalized_direction = normalized(direction);
    if(!finite(origin) || !normalized_direction.has_value()) {
        return std::nullopt;
    }
    const float denominator = dot(normal_, *normalized_direction);
    if(!std::isfinite(denominator) || std::fabs(denominator) < kMinimumVectorLength) {
        return std::nullopt;
    }
    const float distance = dot(normal_, subtract(origin_, origin)) / denominator;
    if(!std::isfinite(distance)) {
        return std::nullopt;
    }
    return add(origin, scale(*normalized_direction, distance));
}

float SurfacePlane::signed_distance(const Vec3 point) const {
    return dot(subtract(point, origin_), normal_);
}

}  // namespace aerial_touch
