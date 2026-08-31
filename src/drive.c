/*
 * Differential-drive recruitment task
 *
 * The communication and decoding stages provide a target coordinate. Implement
 * drive_to_target() so the simulated differential-drive rover reaches the
 * target using valid left and right wheel velocities.
 */

#include <math.h>
#include <stdbool.h>

#define PI_F 3.14159265358979323846f

#define WHEEL_RADIUS      0.15f
#define WHEEL_SEPARATION  0.77f
#define MAX_LINEAR_VELOCITY  1.0f
#define MAX_ANGULAR_VELOCITY 2.0f
#define MAX_WHEEL_VELOCITY  10.0f
#define HEADING_GAIN      1.25f

#define TARGET_TOLERANCE  0.10f
#define DRIVE_DT_SECONDS  0.02f
#define MAX_DRIVE_STEPS   6000

/*
 * Latitude and longitude are normalized local simulation coordinates measured
 * in metres. Latitude is the north axis and longitude is the east axis. The
 * differential-drive rover is planar, so altitude is received but not changed.
 */
struct coordinate {
  float latitude;
  float longitude;
  float altitude;
};

/* Heading is in radians: zero points east and positive rotation is CCW. */
struct rover_state {
  struct coordinate position;
  float heading_rad;
};

struct wheel_velocity {
  float left;
  float right;
};

enum drive_status {
  DRIVE_REACHED_TARGET   =  0,
  DRIVE_INVALID_INPUT    = -1,
  DRIVE_INVALID_COMMAND  = -2,
  DRIVE_MAX_STEPS_EXCEEDED = -3
};

/* Provided simulator helpers. Candidates should not modify these functions. */
static float normalize_angle(float angle);
static bool  apply_wheel_velocities(struct rover_state *rover,
                                     struct wheel_velocity velocity);

/*
 * Candidate task
 * --------------
 * drive_to_target() — proportional heading controller
 *
 * Algorithm:
 *  1. Validate rover and target pointers / finite values.
 *  2. On each time-step:
 *     a. Compute (dx, dy) = target - rover position (east / north).
 *     b. Compute Euclidean distance; stop if <= TARGET_TOLERANCE.
 *     c. Compute desired heading = atan2(dy, dx).
 *     d. Compute heading error = normalize_angle(desired - current).
 *     e. Set angular velocity proportional to heading error (clamped).
 *     f. Set linear velocity = MAX; reduce to 0 when heading error > 90 deg.
 *     g. Convert (linear, angular) -> individual wheel velocities via:
 *          v_left  = (linear - angular * L/2) / R
 *          v_right = (linear + angular * L/2) / R
 *     h. Clamp each wheel velocity to [-MAX_WHEEL_VELOCITY, MAX_WHEEL_VELOCITY].
 *     i. Apply velocities; return DRIVE_INVALID_COMMAND if helper rejects them.
 *  3. Return DRIVE_MAX_STEPS_EXCEEDED if target not reached within MAX_DRIVE_STEPS.
 */
enum drive_status drive_to_target(struct rover_state *rover,
                                   const struct coordinate *target) {
  /* Step 1 – Validate inputs */
  if (rover == NULL || target == NULL) {
    return DRIVE_INVALID_INPUT;
  }
  if (!isfinite(rover->position.latitude)  ||
      !isfinite(rover->position.longitude) ||
      !isfinite(rover->heading_rad)         ||
      !isfinite(target->latitude)           ||
      !isfinite(target->longitude)) {
    return DRIVE_INVALID_INPUT;
  }

  int step;
  for (step = 0; step < MAX_DRIVE_STEPS; step++) {

    /* Step 2a – Direction vector to target (east = longitude, north = latitude) */
    float dx = target->longitude - rover->position.longitude;  /* east  */
    float dy = target->latitude  - rover->position.latitude;   /* north */

    /* Step 2b – Distance check */
    float distance = hypotf(dx, dy);
    if (distance <= TARGET_TOLERANCE) {
      return DRIVE_REACHED_TARGET;
    }

    /* Step 2c – Desired heading (atan2 convention matches rover: 0=east, CCW+) */
    float desired_heading = atan2f(dy, dx);

    /* Step 2d – Heading error with wraparound handling */
    float heading_error = normalize_angle(desired_heading - rover->heading_rad);

    /* Step 2e – Angular velocity proportional to heading error, clamped */
    float angular_vel = HEADING_GAIN * heading_error;
    if (angular_vel >  MAX_ANGULAR_VELOCITY) angular_vel =  MAX_ANGULAR_VELOCITY;
    if (angular_vel < -MAX_ANGULAR_VELOCITY) angular_vel = -MAX_ANGULAR_VELOCITY;

    /* Step 2f – Linear velocity: slow to zero when heading error > 90 degrees */
    float abs_error = (heading_error < 0.0f) ? -heading_error : heading_error;
    float linear_vel = (abs_error > (PI_F / 2.0f)) ? 0.0f : MAX_LINEAR_VELOCITY;

    /* Step 2g – Differential-drive kinematics
     *   v_left  = (V_linear - omega * L/2) / R
     *   v_right = (V_linear + omega * L/2) / R
     */
    float half_sep = WHEEL_SEPARATION / 2.0f;
    float v_left   = (linear_vel - angular_vel * half_sep) / WHEEL_RADIUS;
    float v_right  = (linear_vel + angular_vel * half_sep) / WHEEL_RADIUS;

    /* Step 2h – Clamp to maximum wheel velocity */
    if (v_left  >  MAX_WHEEL_VELOCITY) v_left  =  MAX_WHEEL_VELOCITY;
    if (v_left  < -MAX_WHEEL_VELOCITY) v_left  = -MAX_WHEEL_VELOCITY;
    if (v_right >  MAX_WHEEL_VELOCITY) v_right =  MAX_WHEEL_VELOCITY;
    if (v_right < -MAX_WHEEL_VELOCITY) v_right = -MAX_WHEEL_VELOCITY;

    /* Step 2i – Apply velocities via simulator helper */
    struct wheel_velocity vel = {v_left, v_right};
    if (!apply_wheel_velocities(rover, vel)) {
      return DRIVE_INVALID_COMMAND;
    }
  }

  /* Step 3 – Did not converge within step budget */
  return DRIVE_MAX_STEPS_EXCEEDED;
}

static float normalize_angle(float angle) {
  while (angle >  PI_F) { angle -= 2.0f * PI_F; }
  while (angle < -PI_F) { angle += 2.0f * PI_F; }
  return angle;
}

static bool apply_wheel_velocities(struct rover_state *rover,
                                    struct wheel_velocity velocity) {
  if (!isfinite(velocity.left)  || !isfinite(velocity.right) ||
      fabsf(velocity.left)  > MAX_WHEEL_VELOCITY ||
      fabsf(velocity.right) > MAX_WHEEL_VELOCITY) {
    return false;
  }

  const float linear_velocity =
      WHEEL_RADIUS * (velocity.left + velocity.right) / 2.0f;
  const float angular_velocity =
      WHEEL_RADIUS * (velocity.right - velocity.left) / WHEEL_SEPARATION;

  rover->heading_rad = normalize_angle(
      rover->heading_rad + angular_velocity * DRIVE_DT_SECONDS);
  rover->position.longitude +=
      linear_velocity * cosf(rover->heading_rad) * DRIVE_DT_SECONDS;
  rover->position.latitude  +=
      linear_velocity * sinf(rover->heading_rad) * DRIVE_DT_SECONDS;

  return true;
}