// Analytic sampling for controller-independent trajectory sequences.
//
// This preserves the toolkit's trajectory mathematics: ninth-order time
// scaling, derivatives through snap, and consistent segment boundaries.
//
// The implementation uses bounded storage and does not require ROS, YAML,
// dynamic allocation, or exceptions during onboard execution.

#include "Sampler.hpp"

#include <algorithm>
#include <cmath>

namespace trajectory_generator
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

struct TimeScale
{
  double s;
  double d1;
  double d2;
  double d3;
  double d4;
};

struct ScalarDynamics
{
  double value;
  double d1;
  double d2;
  double d3;
  double d4;
};

bool finite(const Vector3 & vector)
{
  return std::isfinite(vector[0]) &&
         std::isfinite(vector[1]) &&
         std::isfinite(vector[2]);
}

bool finite(const Reference & reference)
{
  return finite(reference.position) &&
         finite(reference.velocity) &&
         finite(reference.acceleration) &&
         finite(reference.jerk) &&
         finite(reference.snap) &&
         std::isfinite(reference.yaw) &&
         std::isfinite(reference.yaw_rate) &&
         std::isfinite(reference.yaw_acceleration);
}

Vector3 add(const Vector3 & first, const Vector3 & second)
{
  return {
    first[0] + second[0],
    first[1] + second[1],
    first[2] + second[2]
  };
}

Vector3 scale(const Vector3 & vector, double factor)
{
  return {
    vector[0] * factor,
    vector[1] * factor,
    vector[2] * factor
  };
}

Reference stationary(const Vector3 & position, double yaw)
{
  Reference reference{};
  reference.position = position;
  reference.yaw = yaw;
  return reference;
}

// Ninth-order C4 interpolation. Derivatives through snap vanish at both
// endpoints, allowing smooth segments to join stationary references.
TimeScale smooth_time_scale(double time_s, double duration_s)
{
  const double t = std::max(0.0, std::min(time_s, duration_s));
  const double u = t / duration_s;

  const double u2 = u * u;
  const double u3 = u2 * u;
  const double u4 = u3 * u;
  const double u5 = u4 * u;
  const double u6 = u5 * u;
  const double u7 = u6 * u;
  const double u8 = u7 * u;
  const double u9 = u8 * u;

  const double s =
    126.0 * u5 - 420.0 * u6 + 540.0 * u7 -
    315.0 * u8 + 70.0 * u9;

  const double du1 =
    630.0 * u4 - 2520.0 * u5 + 3780.0 * u6 -
    2520.0 * u7 + 630.0 * u8;

  const double du2 =
    2520.0 * u3 - 12600.0 * u4 + 22680.0 * u5 -
    17640.0 * u6 + 5040.0 * u7;

  const double du3 =
    7560.0 * u2 - 50400.0 * u3 + 113400.0 * u4 -
    105840.0 * u5 + 35280.0 * u6;

  const double du4 =
    15120.0 * u - 151200.0 * u2 + 453600.0 * u3 -
    529200.0 * u4 + 211680.0 * u5;

  const double inv_t = 1.0 / duration_s;
  const double inv_t2 = inv_t * inv_t;
  const double inv_t3 = inv_t2 * inv_t;
  const double inv_t4 = inv_t3 * inv_t;

  return {
    s,
    du1 * inv_t,
    du2 * inv_t2,
    du3 * inv_t3,
    du4 * inv_t4
  };
}

// Analytical derivatives of amplitude*sin(phase(t)) through fourth order.
ScalarDynamics sinusoid(
  double amplitude,
  double phase,
  double p1,
  double p2,
  double p3,
  double p4)
{
  const double s = std::sin(phase);
  const double c = std::cos(phase);

  return {
    amplitude * s,
    amplitude * c * p1,
    amplitude * (-s * p1 * p1 + c * p2),
    amplitude * (-c * p1 * p1 * p1 - 3.0 * s * p1 * p2 + c * p3),
    amplitude * (
      s * p1 * p1 * p1 * p1 -
      6.0 * c * p1 * p1 * p2 -
      3.0 * s * p2 * p2 -
      4.0 * s * p1 * p3 +
      c * p4
    )
  };
}

Reference sample_line(
  const Segment & segment,
  const Reference & origin,
  const TimeScale & q)
{
  Reference reference{};

  reference.position = add(origin.position, scale(segment.offset, q.s));
  reference.velocity = scale(segment.offset, q.d1);
  reference.acceleration = scale(segment.offset, q.d2);
  reference.jerk = scale(segment.offset, q.d3);
  reference.snap = scale(segment.offset, q.d4);
  reference.yaw = origin.yaw;

  return reference;
}

Reference sample_yaw(
  const Segment & segment,
  const Reference & origin,
  const TimeScale & q)
{
  Reference reference =
    stationary(origin.position, origin.yaw + segment.yaw_offset_rad * q.s);

  reference.yaw_rate = segment.yaw_offset_rad * q.d1;
  reference.yaw_acceleration = segment.yaw_offset_rad * q.d2;

  return reference;
}

Reference sample_figure_eight(
  const Segment & segment,
  const Reference & origin,
  const TimeScale & q)
{
  const double angle = 2.0 * kPi * segment.turns * q.s;
  const double a1 = 2.0 * kPi * segment.turns * q.d1;
  const double a2 = 2.0 * kPi * segment.turns * q.d2;
  const double a3 = 2.0 * kPi * segment.turns * q.d3;
  const double a4 = 2.0 * kPi * segment.turns * q.d4;

  const ScalarDynamics x =
    sinusoid(segment.x_amplitude_m, angle, a1, a2, a3, a4);

  // Twice the x phase produces the planar 1:2 Lissajous curve.
  const ScalarDynamics y =
    sinusoid(
      segment.y_amplitude_m,
      2.0 * angle,
      2.0 * a1,
      2.0 * a2,
      2.0 * a3,
      2.0 * a4
    );

  Reference reference{};

  reference.position = {
    origin.position[0] + x.value,
    origin.position[1] + y.value,
    origin.position[2]
  };

  reference.velocity = {x.d1, y.d1, 0.0};
  reference.acceleration = {x.d2, y.d2, 0.0};
  reference.jerk = {x.d3, y.d3, 0.0};
  reference.snap = {x.d4, y.d4, 0.0};
  reference.yaw = origin.yaw;

  return reference;
}

Reference sample_circle(
  const Segment & segment,
  const Reference & origin,
  const TimeScale & q)
{
  const double angle = 2.0 * kPi * segment.turns * q.s;

  const double w1 = 2.0 * kPi * segment.turns * q.d1;
  const double w2 = 2.0 * kPi * segment.turns * q.d2;
  const double w3 = 2.0 * kPi * segment.turns * q.d3;
  const double w4 = 2.0 * kPi * segment.turns * q.d4;

  const double c = std::cos(angle);
  const double s = std::sin(angle);

  const double r = segment.radius_m;
  const double h =
    segment.type == Segment::Type::Helix ? segment.height_m : 0.0;

  Reference reference{};

  // The segment starts at origin.position. Its circle center is one radius
  // toward negative local-NED x, so entry has no position discontinuity.
  reference.position = {
    origin.position[0] + r * (c - 1.0),
    origin.position[1] + r * s,
    origin.position[2] + h * q.s
  };

  reference.velocity = {
    -r * s * w1,
    r * c * w1,
    h * q.d1
  };

  reference.acceleration = {
    -r * (c * w1 * w1 + s * w2),
    r * (-s * w1 * w1 + c * w2),
    h * q.d2
  };

  reference.jerk = {
    r * (s * w1 * w1 * w1 - 3.0 * c * w1 * w2 - s * w3),
    r * (-c * w1 * w1 * w1 - 3.0 * s * w1 * w2 + c * w3),
    h * q.d3
  };

  reference.snap = {
    r * (
      6.0 * s * w1 * w1 * w2 -
      s * w4 +
      c * w1 * w1 * w1 * w1 -
      4.0 * c * w1 * w3 -
      3.0 * c * w2 * w2
    ),
    r * (
      s * w1 * w1 * w1 * w1 -
      4.0 * s * w1 * w3 -
      3.0 * s * w2 * w2 -
      6.0 * c * w1 * w1 * w2 +
      c * w4
    ),
    h * q.d4
  };

  reference.yaw = origin.yaw;

  return reference;
}

// Select a primitive while keeping sequence logic independent of its shape.
Reference sample_segment(
  const Segment & segment,
  double time_s,
  const Reference & origin)
{
  if (segment.type == Segment::Type::Hold) {
    return stationary(origin.position, origin.yaw);
  }

  if (segment.type == Segment::Type::Step) {
    // A step is instantaneous, including at its segment start.
    return stationary(add(origin.position, segment.offset), origin.yaw);
  }

  const TimeScale q = smooth_time_scale(time_s, segment.duration_s);

  switch (segment.type) {
  case Segment::Type::Line:
    return sample_line(segment, origin, q);

  case Segment::Type::Circle:
  case Segment::Type::Helix:
    return sample_circle(segment, origin, q);

  case Segment::Type::Yaw:
    return sample_yaw(segment, origin, q);

  case Segment::Type::FigureEight:
    return sample_figure_eight(segment, origin, q);

  default:
    return Reference{};
  }
}

}  // namespace

// Reject invalid sequences before sampling. This also protects the bounded
// array against an invalid count and prevents non-finite trajectory inputs.
bool validate(const Sequence & sequence)
{
  if (sequence.count == 0 || sequence.count > kMaxSegments) {
    return false;
  }

  double total_duration_s = 0.0;

  for (std::size_t index = 0; index < sequence.count; ++index) {
    const Segment & segment = sequence.segments[index];

    if (!std::isfinite(segment.duration_s) || segment.duration_s <= 0.0 ||
        !finite(segment.offset) ||
        !std::isfinite(segment.radius_m) ||
        !std::isfinite(segment.height_m) ||
        !std::isfinite(segment.turns) ||
        !std::isfinite(segment.yaw_offset_rad) ||
        !std::isfinite(segment.x_amplitude_m) ||
        !std::isfinite(segment.y_amplitude_m)) {
      return false;
    }

    if ((segment.type == Segment::Type::Circle ||
         segment.type == Segment::Type::Helix) &&
        segment.radius_m <= 0.0) {
      return false;
    }

    if (segment.type == Segment::Type::FigureEight &&
        (segment.x_amplitude_m <= 0.0 ||
         segment.y_amplitude_m <= 0.0 ||
         segment.turns <= 0.0)) {
      return false;
    }

    if (segment.type > Segment::Type::FigureEight) {
      return false;
    }

    total_duration_s += segment.duration_s;

    if (!std::isfinite(total_duration_s)) {
      return false;
    }
  }

  return true;
}

// Evaluate the sequence using configured NED reference coordinates.
// The previous segment's ending position and yaw anchor the next segment.
// At an exact boundary, the ending segment owns the returned sample.
bool sample(
  const Sequence & sequence,
  double time_s,
  const Reference & origin,
  Reference & reference)
{
  if (!validate(sequence) ||
      !std::isfinite(time_s) ||
      !finite(origin.position) ||
      !std::isfinite(origin.yaw)) {
    return false;
  }

  Reference segment_origin = stationary(origin.position, origin.yaw);
  double remaining_s = std::max(0.0, time_s);

  for (std::size_t index = 0; index < sequence.count; ++index) {
    const Segment & segment = sequence.segments[index];

    if (remaining_s <= segment.duration_s) {
      const Reference candidate =
        sample_segment(segment, remaining_s, segment_origin);

      if (!finite(candidate)) {
        return false;
      }

      reference = candidate;
      return true;
    }

    segment_origin =
      sample_segment(segment, segment.duration_s, segment_origin);

    if (!finite(segment_origin)) {
      return false;
    }

    remaining_s -= segment.duration_s;
  }

  // Once every segment has completed, hold the final pose and heading.
  reference = stationary(segment_origin.position, segment_origin.yaw);

  return true;
}

}  // namespace trajectory_generator
