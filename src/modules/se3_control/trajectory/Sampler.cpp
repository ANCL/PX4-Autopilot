// Analytic sampling of Hermite and patterned trajectory segments.
//
// Every channel is p(u) = E(u) + (1 - u)^5 F(u) in normalized time u = t / T
// (see Hermite), so derivatives with respect to t follow exactly from
//
//   d^m p / dt^m = T^-m [E^(m)(u) + sum_i C(m, i) F^(i)(u) g^(m-i)(u)],
//
// with g(u) = (1 - u)^5 (Leibniz rule). Patterns add a planar offset that is
// a sinusoid of the phase channel; their derivatives use the chain rule
// through fourth order.

#include "Trajectory.hpp"

#include <cmath>

namespace trajectory_generator
{
namespace
{

constexpr float kHalfPi = 1.57079632679489662f;

// Value and first four time derivatives of one scalar channel.
struct Dynamics
{
  float d[5];
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

bool finite(const Hermite & hermite)
{
  for (int k = 0; k < 5; ++k) {
    if (!std::isfinite(hermite.end[k]) || !std::isfinite(hermite.correction[k])) {
      return false;
    }
  }

  return true;
}

// Falling factorial k! / (k - order)!.
float falling(int k, int order)
{
  float value = 1.0f;

  for (int i = 0; i < order; ++i) {
    value *= static_cast<float>(k - i);
  }

  return value;
}

// d^order/dx^order of sum_k c[k] x^k (degree 4), by Horner's rule.
float taylor(const float c[5], float x, int order)
{
  float value = 0.0f;

  for (int k = 4; k >= order; --k) {
    value = value * x + falling(k, order) * c[k];
  }

  return value;
}

// Value and first four time derivatives of a Hermite channel.
Dynamics evaluate(const Hermite & hermite, float u, float duration_s)
{
  constexpr float kBinomial[5][5] = {
    {1, 0, 0, 0, 0}, {1, 1, 0, 0, 0}, {1, 2, 1, 0, 0}, {1, 3, 3, 1, 0}, {1, 4, 6, 4, 1}
  };

  // g^(n)(u) = (-1)^n 5!/(5-n)! (1 - u)^(5-n).
  const float w = 1.0f - u;
  const float w2 = w * w;
  const float g[5] {w2 * w2 * w, -5.0f * w2 * w2, 20.0f * w2 * w, -60.0f * w2, 120.0f * w};

  float correction[5];

  for (int i = 0; i < 5; ++i) {
    correction[i] = taylor(hermite.correction, u, i);
  }

  Dynamics result{};
  float scale = 1.0f;
  const float inverse_duration = 1.0f / duration_s;

  for (int order = 0; order < 5; ++order) {
    float value = taylor(hermite.end, u - 1.0f, order);

    for (int i = 0; i <= order; ++i) {
      value += kBinomial[order][i] * correction[i] * g[order - i];
    }

    result.d[order] = value * scale;
    scale *= inverse_duration;
  }

  return result;
}

}  // namespace

// Two-point Hermite form of the degree-9 polynomial with the given end states.
//
// With s_i and e_m the scaled Taylor coefficients x^(i) T^i / i! at u = 0 and
// u = 1, E(u) = sum_m e_m (u - 1)^m. The difference p - E has Taylor
// coefficients d_i = s_i - [u^i] E at u = 0, and F is the degree-4 truncation
// of (p - E)(u) (1 - u)^-5 = sum_j u^j sum_(i<=j) d_i C(j - i + 4, 4).
Hermite hermite(const float start[5], const float end[5], float duration_s)
{
  constexpr float kBinomial[5][5] = {
    {1, 0, 0, 0, 0}, {1, 1, 0, 0, 0}, {1, 2, 1, 0, 0}, {1, 3, 3, 1, 0}, {1, 4, 6, 4, 1}
  };
  constexpr float kInverseSeries[5] {1.0f, 5.0f, 15.0f, 35.0f, 70.0f}; // C(n + 4, 4)

  Hermite result{};
  float scaled_start[5];
  float scale = 1.0f;
  float factorial = 1.0f;

  for (int k = 0; k < 5; ++k) {
    result.end[k] = end[k] * scale / factorial;
    scaled_start[k] = start[k] * scale / factorial;
    scale *= duration_s;
    factorial *= static_cast<float>(k + 1);
  }

  float difference[5];

  for (int i = 0; i < 5; ++i) {
    float coefficient = 0.0f; // [u^i] of E(u) = sum_m e_m (u - 1)^m.

    for (int m = i; m < 5; ++m) {
      coefficient += result.end[m] * kBinomial[m][i] * (((m - i) % 2) ? -1.0f : 1.0f);
    }

    difference[i] = scaled_start[i] - coefficient;
  }

  for (int j = 0; j < 5; ++j) {
    float value = 0.0f;

    for (int i = 0; i <= j; ++i) {
      value += difference[i] * kInverseSeries[j - i];
    }

    result.correction[j] = value;
  }

  return result;
}

namespace
{

// Derivatives of amplitude * sin(phi(t)) from those of phi(t).
Dynamics sinusoid(float amplitude, const Dynamics & phi)
{
  const float s = std::sin(phi.d[0]);
  const float c = std::cos(phi.d[0]);
  const float p1 = phi.d[1];
  const float p2 = phi.d[2];
  const float p3 = phi.d[3];
  const float p4 = phi.d[4];

  return {{
    amplitude * s,
    amplitude * c * p1,
    amplitude * (-s * p1 * p1 + c * p2),
    amplitude * (-c * p1 * p1 * p1 - 3.0f * s * p1 * p2 + c * p3),
    amplitude * (
      s * p1 * p1 * p1 * p1 -
      6.0f * c * p1 * p1 * p2 -
      3.0f * s * p2 * p2 -
      4.0f * s * p1 * p3 +
      c * p4
    )
  }};
}

// phi(t) + offset, or k phi(t) for a frequency multiple k.
Dynamics shifted(Dynamics phi, float offset, float multiple)
{
  for (float & value : phi.d) {
    value *= multiple;
  }

  phi.d[0] += offset;
  return phi;
}

}  // namespace

bool sample_segment(const Segment & segment, float time_s, Reference & reference)
{
  if (!(segment.duration_s > 0.0f) || !std::isfinite(time_s)) {
    return false;
  }

  const float clamped = time_s < 0.0f ? 0.0f :
                        (time_s > segment.duration_s ? segment.duration_s : time_s);
  const float u = clamped / segment.duration_s;

  Dynamics axes[3];

  for (int axis = 0; axis < 3; ++axis) {
    axes[axis] = evaluate(segment.position[axis], u, segment.duration_s);
  }

  if (segment.shape != Segment::Shape::None) {
    const Dynamics phase = evaluate(segment.phase, u, segment.duration_s);
    Dynamics x{};
    Dynamics y{};

    if (segment.shape == Segment::Shape::Circle) {
      // (r cos(phi), r sin(phi)), with cos(phi) = sin(phi + pi/2).
      x = sinusoid(segment.x_amplitude_m, shifted(phase, kHalfPi, 1.0f));
      y = sinusoid(segment.x_amplitude_m, phase);

    } else {
      // 1:2 Lissajous curve through the centre at phase 0.
      x = sinusoid(segment.x_amplitude_m, phase);
      y = sinusoid(segment.y_amplitude_m, shifted(phase, 0.0f, 2.0f));
    }

    for (int order = 0; order < 5; ++order) {
      axes[0].d[order] += x.d[order];
      axes[1].d[order] += y.d[order];
    }
  }

  const Dynamics yaw = evaluate(segment.yaw, u, segment.duration_s);
  Reference candidate{};

  for (int axis = 0; axis < 3; ++axis) {
    candidate.position[axis] = axes[axis].d[0];
    candidate.velocity[axis] = axes[axis].d[1];
    candidate.acceleration[axis] = axes[axis].d[2];
    candidate.jerk[axis] = axes[axis].d[3];
    candidate.snap[axis] = axes[axis].d[4];
  }

  candidate.yaw = yaw.d[0];
  candidate.yaw_rate = yaw.d[1];
  candidate.yaw_acceleration = yaw.d[2];

  if (!finite(candidate)) {
    return false;
  }

  reference = candidate;
  return true;
}

// Reject invalid sequences before sampling. This also protects the bounded
// array against an invalid count and prevents non-finite trajectory inputs.
bool validate(const Sequence & sequence)
{
  if (sequence.count == 0 || sequence.count > kMaxSegments) {
    return false;
  }

  float total_duration_s = 0.0f;

  for (std::size_t index = 0; index < sequence.count; ++index) {
    const Segment & segment = sequence.segments[index];

    if (!std::isfinite(segment.duration_s) || segment.duration_s < 0.01f ||
        !finite(segment.position[0]) ||
        !finite(segment.position[1]) ||
        !finite(segment.position[2]) ||
        !finite(segment.yaw) ||
        !finite(segment.phase) ||
        !std::isfinite(segment.x_amplitude_m) ||
        !std::isfinite(segment.y_amplitude_m) ||
        segment.shape > Segment::Shape::FigureEight) {
      return false;
    }

    if ((segment.shape == Segment::Shape::Circle && segment.x_amplitude_m <= 0.0f) ||
        (segment.shape == Segment::Shape::FigureEight &&
         (segment.x_amplitude_m <= 0.0f || segment.y_amplitude_m <= 0.0f))) {
      return false;
    }

    total_duration_s += segment.duration_s;

    if (!std::isfinite(total_duration_s)) {
      return false;
    }
  }

  return true;
}

std::size_t segment_index(const Sequence & sequence, float time_s)
{
  float remaining_s = time_s > 0.0f ? time_s : 0.0f;

  for (std::size_t index = 0; index < sequence.count && index < kMaxSegments; ++index) {
    if (remaining_s <= sequence.segments[index].duration_s) {
      return index;
    }

    remaining_s -= sequence.segments[index].duration_s;
  }

  return sequence.count;
}

bool sample(const Sequence & sequence, float time_s, Reference & reference)
{
  if (!validate(sequence) || !std::isfinite(time_s)) {
    return false;
  }

  float remaining_s = time_s > 0.0f ? time_s : 0.0f;

  for (std::size_t index = 0; index < sequence.count; ++index) {
    const Segment & segment = sequence.segments[index];

    if (remaining_s <= segment.duration_s) {
      return sample_segment(segment, remaining_s, reference);
    }

    remaining_s -= segment.duration_s;
  }

  // Once every segment has completed, hold the final pose at rest.
  const Segment & last = sequence.segments[sequence.count - 1];
  Reference ending{};

  if (!sample_segment(last, last.duration_s, ending)) {
    return false;
  }

  reference = Reference{};
  reference.position = ending.position;
  reference.yaw = ending.yaw;
  return true;
}

}  // namespace trajectory_generator
