import math
from abc import ABC, abstractmethod

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


PLANETS_COUNT = 3
MODULE_MM = 1
PRESSURE_ANGLE_DEGREE = 20
SF = 3
LOAD_SHARING_FACTOR = 1
STAGES_COUNT = 2
TO_METER = 0.001

input_torque_newton_meters = .4
gear_ratio = 6

sun_teeth_count = 12
sun_face_width_mm = 13
sun_pitch_radius_mm = (MODULE_MM * sun_teeth_count * TO_METER) / 2

ring_teeth_count = (gear_ratio - 1) * sun_teeth_count
ring_face_width_mm = 48

planet_teeth_count = (gear_ratio - 2) * sun_teeth_count / 2
planet_face_width_mm = 8
PLANET_BEARING_WIDTH_MM = 5

carrier_arm_width_mm = 6.3
carrier_arm_thickness_mm = 3

pin_diameter_mm = 5.27
PIN_LENGTH_MM = 7.5
ring_wall_thickness_mm = 8

carrier_hub_radius_mm = 35.1

PLA_STRENGTH = 10e6  # 10 MPa = 10 N/mm² = 10e6 N/m²
SIGMA_ALLOWED_MEGA_PASCAL = PLA_STRENGTH
MAX_SIGMA_ALLOWED_MEGA_PASCAL = SIGMA_ALLOWED_MEGA_PASCAL / SF


class Component:
    @abstractmethod
    def passes_check(self, threshold):
        # This method must be implemented by all subclasses
        pass

    @abstractmethod
    def get_name(self):
        # This method must be implemented by all subclasses
        pass

    def get_fem_loads(self):
        return {}

    def display(self, threshold):
        check = "✅" if self.passes_check(threshold) else "❌"
        print(f"{self.get_name():<14} {check:^7} {self._format_fem():<40}")

    def _format_fem(self):
        fem_dict = self.get_fem_loads()
        if not fem_dict:
            return "-"
        return ", ".join([f"{k}: {v:.2f}" for k, v in fem_dict.items()])


class Gear(Component):
    # Lewis form factor lookup for 20° full-depth involute external spur gears
    EXTERNAL_LEWIS_20_TABLE = {
        10: 0.201, 11: 0.226, 12: 0.245, 13: 0.264, 14: 0.276,
        15: 0.289, 16: 0.295, 17: 0.302, 18: 0.308, 19: 0.314,
        20: 0.320, 22: 0.330, 24: 0.337, 26: 0.344, 28: 0.352,
        30: 0.358, 32: 0.364, 34: 0.370, 36: 0.377, 38: 0.383,
        40: 0.389, 45: 0.399, 50: 0.408, 55: 0.415, 60: 0.421,
        65: 0.425, 70: 0.429, 75: 0.433, 80: 0.436, 90: 0.442,
        100: 0.446, 150: 0.458, 200: 0.463, 300: 0.471
    }

    def __init__(self, teeth_count, face_width_mm, effective_force):
        self.teeth_count = teeth_count
        self.face_width_mm = face_width_mm
        self.pitch_radius_mm = (MODULE_MM * self.teeth_count) / 2
        self.effective_force = effective_force

    def passes_check(self, threshold):
        return self._calculate_bending_stress() < threshold

    def get_name(self):
        return "Sun"

    def _calculate_bending_stress(self):
        # σ = F / (b * m * y)
        lewis_y = self._get_lewis_form_factor(self.teeth_count)
        return self.effective_force / (self.face_width_mm * MODULE_MM * lewis_y * TO_METER)

    def _get_lewis_form_factor(self, teeth):
        """
        Return Lewis form factor for external spur gear with 20° full-depth involute.
        If exact teeth not in table, interpolate linearly between nearest keys.
        """

        if teeth in self.EXTERNAL_LEWIS_20_TABLE:
            return self.EXTERNAL_LEWIS_20_TABLE[teeth]

        # simple linear interpolation between nearest entries
        keys = sorted(self.EXTERNAL_LEWIS_20_TABLE.keys())
        for i in range(len(keys) - 1):
            low, high = keys[i], keys[i + 1]
            if low < teeth < high:
                y_low = self.EXTERNAL_LEWIS_20_TABLE[low]
                y_high = self.EXTERNAL_LEWIS_20_TABLE[high]
                frac = (teeth - low) / (high - low)
                return y_low + (y_high - y_low) * frac

        # fallback to nearest
        return self.EXTERNAL_LEWIS_20_TABLE[keys[-1] if teeth > keys[-1] else keys[0]]


class SecondaryGear(Gear):
    def __init__(self, teeth_count, face_width_mm, effective_force):
        super().__init__(teeth_count, face_width_mm, effective_force)
        self.effective_force = self.effective_force / PLANETS_COUNT

    def get_name(self):
        return "Planet"


class Ring(SecondaryGear):
    def __init__(self, teeth_count, face_width_mm, effective_force, thickness):
        super().__init__(teeth_count, face_width_mm, effective_force)
        self.thickness = thickness
        self.radial_force = self.effective_force * math.tan(math.radians(PRESSURE_ANGLE_DEGREE))

    def passes_check(self, threshold):
        return self._calculate_bending_stress() < threshold and self._calculate_ovalization() < threshold

    def get_name(self):
        return "Ring"

    def get_fem_loads(self):
        """
        Returns radial load and tooth load for FEM.
        """
        sigma_ovalization = self._calculate_ovalization()
        sigma_tooth = self._calculate_bending_stress()
        return {
            "F_radial": self.radial_force,
            "sigma_ovalization": sigma_ovalization,
            "sigma_tooth": sigma_tooth
        }

    def _calculate_bending_stress(self):
        y_ext = self._get_lewis_form_factor(self.teeth_count)
        y_internal = 1.3 * y_ext
        return self.effective_force / (self.face_width_mm * MODULE_MM * y_internal * TO_METER)

    def _calculate_ovalization(self):
        # σ_ring ≈ (F_r,p · r_r) / (t_ring · b)
        return (self.radial_force * self.pitch_radius_mm * TO_METER) / (self.thickness * self.face_width_mm * TO_METER)


class Pin(Component):
    def __init__(self, planet, diameter_mm, length_mm):
        self.planet = planet
        self.diameter_mm = diameter_mm
        self.length_mm = length_mm

    def passes_check(self, threshold):
        return self._calculate_von_mises() < threshold and self._calculate_bearing() < threshold

    def get_name(self):
        return "Pin"

    def get_fem_loads(self):
        """
        Returns bending moment and shear for FEM.
        """
        tau_pin = self._calculate_shear()
        M_pin = self._calculate_moment()
        sigma_bending = self._calculate_sigma(M_pin)
        return {
            "M_bending": M_pin,
            "sigma_bending": sigma_bending,
            "tau_shear": tau_pin
        }

    def _calculate_shear(self):
        # F_t,p / (π·d_pin²/4)
        return self.planet.effective_force / (math.pi * math.pow(self.diameter_mm * TO_METER, 2) / 4)

    def _calculate_bending(self):
        moment = self._calculate_moment()
        return self._calculate_sigma(moment)

    def _calculate_moment(self):
        # Cantilever bending: force acts at free end
        cantilever_length_mm = self.length_mm - PLANET_BEARING_WIDTH_MM
        return self.planet.effective_force * (cantilever_length_mm * TO_METER)

    def _calculate_sigma(self, moment):
        # Cantilever bending stress
        d_m = self.diameter_mm * TO_METER
        return 32 * moment / (math.pi * math.pow(d_m, 3))

    def _calculate_von_mises(self):
        moment = self._calculate_moment()
        sigma = self._calculate_sigma(moment)
        tau = self._calculate_shear()
        return math.sqrt(sigma ** 2 + 3 * tau ** 2)

    def _calculate_bearing(self):
        # Bearing stress over embedded length
        area_embed = self.diameter_mm * TO_METER * PLANET_BEARING_WIDTH_MM * TO_METER
        return self.planet.effective_force / area_embed


class CarrierArm(Component):
    def __init__(self, planet, width, thickness, carrier_hub_torque):
        self.planet = planet
        self.width = width
        self.thickness = thickness
        self.carrier_hub_torque = carrier_hub_torque

    def passes_check(self, threshold):
        return self._calculate_shear() < threshold and self._calculate_bending() < threshold

    def get_name(self):
        return "CarrierArm"

    def get_fem_loads(self):
        """
        Returns bending moment and torsion for FEM.
        """
        M_bending = self._calculate_moment()
        sigma_bending = self._calculate_sigma_bending(M_bending)
        tau_torsion = (2 * self.carrier_hub_torque) / (
                math.pi * math.pow(self.planet.pitch_radius_mm * TO_METER, 3))  # simple solid shaft
        return {
            "M_bending": M_bending,
            "sigma_bending": sigma_bending,
            "tau_torsion": tau_torsion
        }

    def _calculate_shear(self):
        # τ_arm = F_t,p / (w · t)
        return self.planet.effective_force / (self.width * self.thickness)

    def _calculate_bending(self):
        M_arm = self._calculate_moment()
        return self._calculate_sigma_bending(M_arm)

    def _calculate_moment(self):
        # M_arm = F_t,p · r_p
        return self.planet.effective_force * self.planet.pitch_radius_mm * TO_METER

    def _calculate_sigma_bending(self, M_arm):
        # I = w·t³ / 12
        # c = t / 2
        # σ_arm = M_arm · c / I
        I = self.width * math.pow(self.thickness, 3) / 12
        c = self.thickness / 2
        return M_arm * c / I


class CarrierHub(Component):
    def __init__(self, output_torque, radius_meter):
        self.output_torque = output_torque
        self.radius_meter = radius_meter

    def passes_check(self, threshold):
        return self._calculate_shear() < threshold

    def get_name(self):
        return "CarrierHub"

    def _calculate_shear(self):
        return (2 * self.output_torque) / (math.pi * self.radius_meter ** 3)


class Stage:
    def __init__(self, index, sun, planet, ring, pin, carrier_arm, carrier_hub):
        self.index = index
        self.components = [sun, planet, ring, pin, carrier_arm, carrier_hub]

    def display(self):
        print(f"Stage {self.index} results:")
        print(f"Component\tPass\tFem\n")
        for component in self.components:
            component.display(MAX_SIGMA_ALLOWED_MEGA_PASCAL)
        print()

    def check_passed(self):
        return all([c.passes_check(MAX_SIGMA_ALLOWED_MEGA_PASCAL) for c in self.components])


current_input_torque = input_torque_newton_meters

for i in range(1, STAGES_COUNT + 1):

    # 1. Calculate effective tangential force for this stage
    effective_force = (current_input_torque / sun_pitch_radius_mm) * LOAD_SHARING_FACTOR

    # 2. Create new component instances for this stage
    sun = Gear(sun_teeth_count, sun_face_width_mm, effective_force)
    planet = SecondaryGear(planet_teeth_count, planet_face_width_mm, effective_force)
    ring = Ring(ring_teeth_count, ring_face_width_mm, effective_force, ring_wall_thickness_mm)
    pin = Pin(planet, pin_diameter_mm, PIN_LENGTH_MM)

    # 3. Output torque for this stage
    stage_output_torque = current_input_torque * gear_ratio

    # 4. Carrier components
    carrier_arm = CarrierArm(planet, carrier_arm_width_mm, carrier_arm_thickness_mm, stage_output_torque)
    carrier_hub = CarrierHub(stage_output_torque, carrier_hub_radius_mm)

    # 5. Create stage and store it
    stage = Stage(i, sun, planet, ring, pin, carrier_arm, carrier_hub)

    # 6. Display results
    stage.display()

    # 7. Stop if any component fails
    if not stage.check_passed():
        print(f"Stage {i} failed stress check ❌.")
        break
    else:
        if i == STAGES_COUNT:
            print(f"All {STAGES_COUNT} stages passed successfully! ✅")
        else:
            print(f"Stage {i} passed stress check ✅. Moving to next stage...\n")

    # 8. Prepare torque for next stage
    current_input_torque = stage_output_torque
