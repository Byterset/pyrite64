/**
 * @copyright 2024 - Max Bebök
 * @license MIT
 */
#pragma once

#include <cmath>
#include <t3d/t3dmath.h>

namespace P64::CollNew {

  constexpr float EPSILON = 0.000001f;

  inline fm_vec3_t vec3(float x, float y, float z) {
    return fm_vec3_t{{x, y, z}};
  }

  inline fm_vec3_t vec3Zero() {
    return fm_vec3_t{{0.0f, 0.0f, 0.0f}};
  }

  inline fm_vec3_t vec3Right() {
    return fm_vec3_t{{1.0f, 0.0f, 0.0f}};
  }

  inline fm_vec3_t vec3Up() {
    return fm_vec3_t{{0.0f, 1.0f, 0.0f}};
  }

  inline fm_vec3_t vec3Forward() {
    return fm_vec3_t{{0.0f, 0.0f, 1.0f}};
  }

  inline float vec3Dot(const fm_vec3_t &a, const fm_vec3_t &b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
  }

  inline float vec3MagSqrd(const fm_vec3_t &v) {
    return vec3Dot(v, v);
  }

  inline float vec3Mag(const fm_vec3_t &v) {
    return sqrtf(vec3MagSqrd(v));
  }

  inline float vec3DistSqrd(const fm_vec3_t &a, const fm_vec3_t &b) {
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    float dz = a.z - b.z;
    return dx * dx + dy * dy + dz * dz;
  }

  inline fm_vec3_t vec3Negate(const fm_vec3_t &v) {
    return fm_vec3_t{{-v.x, -v.y, -v.z}};
  }

  inline fm_vec3_t vec3Scale(const fm_vec3_t &v, float s) {
    return fm_vec3_t{{v.x * s, v.y * s, v.z * s}};
  }

  inline fm_vec3_t vec3Add(const fm_vec3_t &a, const fm_vec3_t &b) {
    return fm_vec3_t{{a.x + b.x, a.y + b.y, a.z + b.z}};
  }

  inline fm_vec3_t vec3Sub(const fm_vec3_t &a, const fm_vec3_t &b) {
    return fm_vec3_t{{a.x - b.x, a.y - b.y, a.z - b.z}};
  }

  inline fm_vec3_t vec3AddScaled(const fm_vec3_t &a, const fm_vec3_t &b, float scale) {
    return fm_vec3_t{{a.x + b.x * scale, a.y + b.y * scale, a.z + b.z * scale}};
  }

  inline fm_vec3_t vec3Cross(const fm_vec3_t &a, const fm_vec3_t &b) {
    return fm_vec3_t{{
      a.y * b.z - a.z * b.y,
      a.z * b.x - a.x * b.z,
      a.x * b.y - a.y * b.x
    }};
  }

  inline fm_vec3_t vec3Normalize(const fm_vec3_t &v) {
    float magSq = vec3MagSqrd(v);
    if(magSq < EPSILON * EPSILON) {
      return vec3Zero();
    }
    float invMag = 1.0f / sqrtf(magSq);
    return vec3Scale(v, invMag);
  }

  inline fm_vec3_t vec3Perpendicular(const fm_vec3_t &a) {
    if(fabsf(a.x) > fabsf(a.z)) {
      return vec3Cross(a, vec3Forward());
    }
    return vec3Cross(a, vec3Right());
  }

  /// Computes the vector triple product: (a × b) × c = b(a·c) - a(b·c)
  inline fm_vec3_t vec3TripleProduct(const fm_vec3_t &a, const fm_vec3_t &b, const fm_vec3_t &c) {
    float ac = vec3Dot(a, c);
    float bc = vec3Dot(b, c);
    return fm_vec3_t{{
      b.x * ac - a.x * bc,
      b.y * ac - a.y * bc,
      b.z * ac - a.z * bc
    }};
  }

  inline bool vec3IsZero(const fm_vec3_t &v) {
    return v.x == 0.0f && v.y == 0.0f && v.z == 0.0f;
  }

  inline fm_vec3_t vec3Min(const fm_vec3_t &a, const fm_vec3_t &b) {
    return fm_vec3_t{{fminf(a.x, b.x), fminf(a.y, b.y), fminf(a.z, b.z)}};
  }

  inline fm_vec3_t vec3Max(const fm_vec3_t &a, const fm_vec3_t &b) {
    return fm_vec3_t{{fmaxf(a.x, b.x), fmaxf(a.y, b.y), fmaxf(a.z, b.z)}};
  }

  inline fm_vec3_t vec3Abs(const fm_vec3_t &v) {
    return fm_vec3_t{{fabsf(v.x), fabsf(v.y), fabsf(v.z)}};
  }

  // ----- Barycentric Coordinates -----

  inline float calculateLerp(const fm_vec3_t &a, const fm_vec3_t &b, const fm_vec3_t &point) {
    auto v0 = vec3Sub(b, a);
    float denom = vec3MagSqrd(v0);
    if(denom < EPSILON * EPSILON) return 0.5f;
    auto offset = vec3Sub(point, a);
    return vec3Dot(offset, v0) / denom;
  }

  inline fm_vec3_t calculateBarycentricCoords(
    const fm_vec3_t &a, const fm_vec3_t &b, const fm_vec3_t &c, const fm_vec3_t &point
  ) {
    auto v0 = vec3Sub(b, a);
    auto v1 = vec3Sub(c, a);
    auto v2 = vec3Sub(point, a);

    float d00 = vec3Dot(v0, v0);
    float d01 = vec3Dot(v0, v1);
    float d11 = vec3Dot(v1, v1);
    float d20 = vec3Dot(v2, v0);
    float d21 = vec3Dot(v2, v1);

    float denom = d00 * d11 - d01 * d01;

    if(fabsf(denom) < EPSILON) {
      fm_vec3_t result;
      if(d00 > d11) {
        result.y = calculateLerp(a, b, point);
        result.x = 1.0f - result.y;
        result.z = 0.0f;
      } else {
        result.z = calculateLerp(a, c, point);
        result.x = 1.0f - result.z;
        result.y = 0.0f;
      }
      return result;
    }

    float denomInv = 1.0f / denom;
    fm_vec3_t result;
    result.y = (d11 * d20 - d01 * d21) * denomInv;
    result.z = (d00 * d21 - d01 * d20) * denomInv;
    result.x = 1.0f - result.y - result.z;
    return result;
  }

  inline fm_vec3_t evaluateBarycentricCoords(
    const fm_vec3_t &a, const fm_vec3_t &b, const fm_vec3_t &c, const fm_vec3_t &bary
  ) {
    auto result = vec3Scale(a, bary.x);
    if(bary.y > EPSILON) {
      result = vec3AddScaled(result, b, bary.y);
      result = vec3AddScaled(result, c, bary.z);
    }
    return result;
  }

  // ----- Plane -----

  struct Plane {
    fm_vec3_t normal{};
    float d{0.0f};
  };

  inline Plane planeFromNormalAndPoint(const fm_vec3_t &normal, const fm_vec3_t &point) {
    return Plane{normal, -vec3Dot(normal, point)};
  }

  inline bool planeRayIntersection(const Plane &plane, const fm_vec3_t &rayOrigin, const fm_vec3_t &rayDir, float &outDistance) {
    float normalDot = vec3Dot(plane.normal, rayDir);
    if(fabsf(normalDot) < EPSILON) return false;
    outDistance = -(vec3Dot(rayOrigin, plane.normal) + plane.d) / normalDot;
    return true;
  }

  // ----- Additional vector utilities -----

  inline fm_vec3_t vec3Project(const fm_vec3_t &v, const fm_vec3_t &onto) {
    float d = vec3Dot(v, onto);
    float m = vec3MagSqrd(onto);
    if(m < EPSILON) return vec3Zero();
    return vec3Scale(onto, d / m);
  }

  inline fm_vec3_t vec3ClampMag(const fm_vec3_t &v, float maxMag) {
    float magSq = vec3MagSqrd(v);
    if(magSq > maxMag * maxMag) {
      return vec3Scale(v, maxMag / sqrtf(magSq));
    }
    return v;
  }

  inline void vec3CalculateTangents(const fm_vec3_t &normal, fm_vec3_t &tangentU, fm_vec3_t &tangentV) {
    if(fabsf(normal.x) > fabsf(normal.z)) {
      float invLen = 1.0f / sqrtf(normal.x * normal.x + normal.y * normal.y);
      tangentU = vec3(-normal.y * invLen, normal.x * invLen, 0.0f);
    } else {
      float invLen = 1.0f / sqrtf(normal.y * normal.y + normal.z * normal.z);
      tangentU = vec3(0.0f, -normal.z * invLen, normal.y * invLen);
    }
    tangentV = vec3Cross(normal, tangentU);
  }

  // ----- Quaternion utilities -----

  inline fm_quat_t quatConjugate(const fm_quat_t &q) {
    return {-q.x, -q.y, -q.z, q.w};
  }

  inline fm_vec3_t quatRotateVec(const fm_quat_t &q, const fm_vec3_t &v) {
    float qx = q.x, qy = q.y, qz = q.z, qw = q.w;
    float tx = 2.0f * (qy * v.z - qz * v.y);
    float ty = 2.0f * (qz * v.x - qx * v.z);
    float tz = 2.0f * (qx * v.y - qy * v.x);
    return vec3(v.x + qw * tx + (qy * tz - qz * ty),
                v.y + qw * ty + (qz * tx - qx * tz),
                v.z + qw * tz + (qx * ty - qy * tx));
  }

  inline float quatDot(const fm_quat_t &a, const fm_quat_t &b) {
    return a.x*b.x + a.y*b.y + a.z*b.z + a.w*b.w;
  }

  inline fm_quat_t quatNormalize(const fm_quat_t &q) {
    float mag = sqrtf(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
    if(mag < EPSILON) return {0, 0, 0, 1};
    float inv = 1.0f / mag;
    return {q.x*inv, q.y*inv, q.z*inv, q.w*inv};
  }

  inline fm_quat_t quatApplyAngularVelocity(const fm_quat_t &q, const fm_vec3_t &omega, float dt) {
    float hdt = 0.5f * dt;
    fm_quat_t dq;
    dq.x = hdt * (omega.x * q.w + omega.y * q.z - omega.z * q.y);
    dq.y = hdt * (omega.y * q.w + omega.z * q.x - omega.x * q.z);
    dq.z = hdt * (omega.z * q.w + omega.x * q.y - omega.y * q.x);
    dq.w = hdt * (-omega.x * q.x - omega.y * q.y - omega.z * q.z);
    return quatNormalize({q.x + dq.x, q.y + dq.y, q.z + dq.z, q.w + dq.w});
  }

} // namespace P64::CollNew
