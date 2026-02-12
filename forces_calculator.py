import math

# | Name                | Symbol | Value | Units |
# | ------------------- | ------ | ----- | ----- |
# | Input torque        | `T_in` |       | Nm    |
# | Sun pitch radius    | `r_s`  |       | m     |
# | Planet pitch radius | `r_p`  |       | m     |
# | Ring pitch radius   | `r_r`  |       | m     |
# | Number of planets   | `N`    |       | –     |
# | Face width          | `b`    |       | m     |
# | Module              | `m`    | 0.001 | m     |
# | Pressure angle      | `α`    | 20    | deg   |
# | Carrier arm width     | `w`      |       |
# | Carrier arm thickness | `t`      |       |
# | Carrier hub radius    | `r_hub`  |       |
# | Pin diameter          | `d_pin`  |       |
# | Ring wall thickness   | `t_ring` |       |

# Tangential force
# F_t = T_in / r_s

# Per planet:
# F_t,p = F_t / N

# Radial force
# F_r,p = F_t,p · tan(α)

# Sun: F_t_effective = F_t
# Planet / ring: F_t_effective = F_t,p


input_torque_newton_meters = .4
gear_ratio = 6

sun_teeth_count = 12
sun_face_width = 13.3 * 0.001

ring_teeth_count = (gear_ratio - 1) * sun_teeth_count
ring_face_width = 48.3 * 0.001

planet_teeth_count = (gear_ratio - 2) * sun_teeth_count / 2
planet_face_width = 8.3 * 0.001

carrier_arm_width_meter = 6.3 * 0.001
carrier_arm_thickness_meter = 3 * 0.001
pin_diameter_meter = 5.27 * 0.001
ring_wall_thickness_meter = 8 * 0.001

PLANETS_COUNT = 3
MODULE_METER = 10.001
PRESSURE_ANGLE_DEGREE = 20
SF = 3

PLA_STRENGTH = 10
SIGMA_ALLOWED_MEGA_PASCAL = PLA_STRENGTH
MAX_SIGMA_ALLOWED_MEGA_PASCAL = SIGMA_ALLOWED_MEGA_PASCAL / SF

class Gear:
    # Lewis form factor lookup for 20° full-depth involute external spur gears
    LEWIS_20_TABLE = {
        10: 0.201, 11: 0.226, 12: 0.245, 13: 0.264, 14: 0.276,
        15: 0.289, 16: 0.295, 17: 0.302, 18: 0.308, 19: 0.314,
        20: 0.320, 22: 0.330, 24: 0.337, 26: 0.344, 28: 0.352,
        30: 0.358, 32: 0.364, 34: 0.370, 36: 0.377, 38: 0.383,
        40: 0.389, 45: 0.399, 50: 0.408, 55: 0.415, 60: 0.421,
        65: 0.425, 70: 0.429, 75: 0.433, 80: 0.436, 90: 0.442,
        100: 0.446, 150: 0.458, 200: 0.463, 300: 0.471
    }

    def __init__(self, teeth_count, face_width, module, torque):
        self.teeth_count = teeth_count
        self.face_width = face_width
        self.module = module
        self.pitch_radius_meter = teeth_count / 2 * 0.001
        self.effective_force = torque / self.pitch_radius_meter

    def passes_check(self, threshold):
        return self._calculate_bending_stress() < threshold

    def _calculate_bending_stress(self):
        lewis_y = self._get_lewis_form_factor(self.teeth_count)
        return self.effective_force / (self.face_width * self.module * lewis_y)

    def _get_lewis_form_factor(self, teeth):
        """
        Return Lewis form factor for external spur gear with 20° full-depth involute.
        If exact teeth not in table, interpolate linearly between nearest keys.
        """

        if teeth in self.LEWIS_20_TABLE:
            return self.LEWIS_20_TABLE[teeth]

        # simple linear interpolation between nearest entries
        keys = sorted(self.LEWIS_20_TABLE.keys())
        for i in range(len(keys) - 1):
            low, high = keys[i], keys[i + 1]
            if low < teeth < high:
                y_low = self.LEWIS_20_TABLE[low]
                y_high = self.LEWIS_20_TABLE[high]
                frac = (teeth - low) / (high - low)
                return y_low + (y_high - y_low) * frac

        # fallback to nearest
        return self.LEWIS_20_TABLE[keys[-1] if teeth > keys[-1] else keys[0]]

class SecondaryGear(Gear):
    def __init__(self, teeth_count, face_width, module, torque):
        super().__init__(teeth_count, face_width, module, torque)
        self.effective_force = self.effective_force / PLANETS_COUNT
        self.radial_force = self.effective_force * math.tan(math.radians(PRESSURE_ANGLE_DEGREE))

    @property
    def pitch_radius_meter(self):
        return self.pitch_radius_meter
    @property
    def effective_force(self):
        return self.effective_force
    @property
    def face_width(self):
        return self.face_width
    
class Ring(SecondaryGear):
    def __init__(self, teeth_count, face_width, module, torque, thickness):
        super().__init__(teeth_count, face_width, module, torque)
        self.thickness = thickness

    def passes_check(self, threshold):
        return super().passes_check(threshold) and self._calculate_ovalization() < threshold

    def _calculate_ovalization(self):
        # σ_ring ≈ (F_r,p · r_r) / (t_ring · b)
        return (self.effective_force * self.pitch_radius_meter) / (self.thickness * self.face_width)

class Pin:
    def __init__(self, planet, diameter, length):
        self.planet = planet
        self.diameter = diameter
        self.length = length

    def passes_check(self, threshold):
        return self._calculate_shear() < threshold and self._calculate_bending() < threshold

    def _calculate_shear(self):
        # F_t,p / (π·d_pin²/4)
        return self.planet.effective_force / (math.pi * math.pow(self.diameter, 2) / 4)

    def _calculate_bending(self):
        # M_pin = F_t,p · eccentricity
        # σ_pin = 32·M_pin / (π·d_pin³)

        eccentricity = (self.planet.face_width / 2) - (self.length / 2)
        moment = self.planet.effective_force * eccentricity
        return (32 * moment) / (math.pi * math.pow(self.diameter, 3))

class CarrierArm:
    def __init__(self, planet, width, thickness):
        self.planet = planet
        self.width = width
        self.thickness = thickness

    def passes_check(self, threshold):
        return self._calculate_shear() < threshold and self._calculate_bending() < threshold

    def _calculate_shear(self):
        # τ_arm = F_t,p / (w · t)
        return self.planet.effective_force / (self.width * self.thickness)

    def _calculate_bending(self):
        # M_arm = F_t,p · r_p
        # I = w·t³ / 12
        # c = t / 2
        # σ_arm = M_arm · c / I

        M_arm = self.planet.effective_force * self.planet.pitch_radius_meter
        I = self.width * math.pow(self.thickness, 3) / 12
        c = self.thickness / 2
        return M_arm * c / I



sun = Gear(sun_teeth_count, sun_face_width, MODULE_METER, input_torque_newton_meters)
planet = SecondaryGear(planet_teeth_count, planet_face_width, MODULE_METER, input_torque_newton_meters)
ring = Ring(ring_teeth_count, ring_face_width, MODULE_METER, input_torque_newton_meters, ring_wall_thickness_meter)

pin = Pin(planet, pin_diameter_meter, 5 * 0.001)
carrierArm = CarrierArm(input_torque_newton_meters, carrier_arm_width_meter, carrier_arm_thickness_meter,
                        planet.pitch_radius_meter)

components = [sun, planet, ring, pin, carrierArm]

for c in components:
    c.passes_check(MAX_SIGMA_ALLOWED_MEGA_PASCAL)
