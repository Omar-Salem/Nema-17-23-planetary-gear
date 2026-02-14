import math
from abc import ABC, abstractmethod



MOTOR_TORQUE_NEWTON_MM = 400
MATERIAL_STRENGTH_MEGA_PASCAL = 38  # N/mm²
SAFETY_FACTOR = 3

GEAR_RATIO = 6

PLANETS_COUNT = 3
MODULE_MM = 1
PRESSURE_ANGLE_DEGREE = 20
LOAD_SHARING_FACTOR = 1
STAGES_COUNT = 2

SUN_TEETH_COUNT = 12
SUN_FACE_WIDTH_MM = 13
SUN_PITCH_RADIUS_MM = (MODULE_MM * SUN_TEETH_COUNT ) / 2

RING_TEETH_COUNT = (GEAR_RATIO - 1) * SUN_TEETH_COUNT
RING_FACE_WIDTH_MM = 48

PLANET_TEETH_COUNT = (GEAR_RATIO - 2) * SUN_TEETH_COUNT / 2
PLANET_FACE_WIDTH_MM = 8

CARRIER_ARM_WIDTH_MM = 6.3
CARRIER_ARM_THICKNESS_MM = 8

PIN_DIAMETER_MM = 5.27
PIN_LENGTH_MM = 5
PIN_FILLET_RADIUS_MM = .5
RING_WALL_THICKNESS_MM = 8

CARRIER_HUB_RADIUS_MM = 35.1

MAX_SIGMA_ALLOWED_MEGA_PASCAL = MATERIAL_STRENGTH_MEGA_PASCAL / SAFETY_FACTOR


class Component:

    @abstractmethod
    def get_name(self):
        # This method must be implemented by all subclasses
        pass

    def get_fem_loads(self):
        return {}

    @abstractmethod
    def get_governing_stress(self):
        """
        Return the worst-case stress for this component.
        Must be implemented by subclasses.
        """
        pass

    def get_margin_data(self, threshold):
        sigma = self.get_governing_stress()
        utilization = sigma / threshold
        margin_of_safety = (threshold / sigma) - 1 if sigma != 0 else float("inf")
        delta = threshold - sigma

        return sigma, utilization, margin_of_safety, delta

    def display(self, threshold):
        sigma, util, mos, delta = self.get_margin_data(threshold)
        check = "✅" if util < 1 else "❌"
        fem_output = self._format_fem() if util < 1 else "-"

        if mos > 9999:
            mos_str = " >9999"
        else:
            mos_str = f"{mos:7.2f}"

        print(
            f"{self.get_name():<15}"
            f"{check:<7}"
            f"σ MPa={sigma :<6.2f} "
            f"U={util:<8.2f} "
            f"MoS={mos_str:<9} "
            f"   {fem_output}"
        )

    def _format_fem(self):
        fem_dict = self.get_fem_loads()
        if not fem_dict:
            return "-"
        return ", ".join([f"{k}: {v:.2e}" for k, v in fem_dict.items()])


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
        return self.effective_force / (self.face_width_mm * MODULE_MM * lewis_y )

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

    def get_governing_stress(self):
        return self._calculate_bending_stress()


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
        return self.effective_force / (self.face_width_mm * MODULE_MM * y_internal )

    def _calculate_ovalization(self):
        # σ_ring ≈ (F_r,p · r_r) / (t_ring · b)
        return (self.radial_force * self.pitch_radius_mm ) / (self.thickness * self.face_width_mm )

    def get_governing_stress(self):
        return max(
            self._calculate_bending_stress(),
            self._calculate_ovalization()
        )


class Pin(Component):
    def __init__(self, planet, diameter_mm, length_mm, fillet_radius_mm, bolt_diameter_mm=None):
        self.planet = planet
        self.diameter_mm = diameter_mm
        self.length_mm = length_mm
        self.fillet_radius_mm = fillet_radius_mm
        self.bolt_diameter_mm = bolt_diameter_mm

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

    def _calculate_bending(self):
        moment = self._calculate_moment()
        return self._calculate_sigma(moment)

    def _calculate_shear(self):
        """
        Calculates peak transverse shear stress for a solid cylinder.
        Formula: (4 * V) / (3 * Area)
        """
        V = self.planet.effective_force  # Total force
        area = (math.pi * math.pow(self.diameter_mm , 2)) / 4
        return (4 * V) / (3 * area)

    def _calculate_moment(self):
        """
        Calculates max moment for a Cantilever with Uniformly Distributed Load (UDL).
        Formula: M = (Force * Length) / 2
        """
        force = self.planet.effective_force
        # For UDL, max moment at the root is FL/2
        return (force * self.length_mm ) / 2

    def _calculate_sigma(self, moment):
        """
        Bending stress using combined inertia of pin + bolt if present.
        """
        d_pin_m = self.diameter_mm 
        I_pin = (math.pi * math.pow(d_pin_m, 4)) / 64

        if self.bolt_diameter_mm:
            d_bolt_m = self.bolt_diameter_mm 
            I_bolt = (math.pi * math.pow(d_bolt_m, 4)) / 64
            I_eff = I_pin + I_bolt
            c_max = max(d_pin_m, d_bolt_m) / 2
        else:
            I_eff = I_pin
            c_max = d_pin_m / 2

        return moment * c_max / I_eff

    def _get_kt(self):
        """
        Estimates the stress concentration factor (Kt) for a stepped shaft in bending.
        Based on r/d (fillet radius / diameter).
        """
        r_d = self.fillet_radius_mm / self.diameter_mm
        if r_d < 0.01: return 3.0  # Very sharp
        if r_d < 0.05: return 2.2  # Typical machined corner
        if r_d < 0.1:  return 1.7  # Small fillet
        return 1.5  # Generous fillet

    def _calculate_von_mises(self):
        moment = self._calculate_moment()
        sigma_nominal = self._calculate_sigma(moment)
        tau = self._calculate_shear()

        # Apply Kt only to the pin (fillet), assume bolt is smooth and concentric
        kt = self._get_kt()
        sigma_max = sigma_nominal * kt

        # Von Mises combining bending + shear
        return math.sqrt(sigma_max ** 2 + 3 * (tau ** 2))

    def _calculate_bearing(self):
        """
        Bearing stress over the projected area.
        """
        area_proj = (self.diameter_mm ) * (self.length_mm )
        return self.planet.effective_force / area_proj

    def get_governing_stress(self):
        return max(
            self._calculate_von_mises(),
            self._calculate_bearing()
        )


# class CarrierArm(Component):
#     def __init__(self, planet, width_mm, thickness_mm, carrier_hub_torque):
#         self.planet = planet
#         self.width_mm = width_mm
#         self.thickness_mm = thickness_mm
#         self.carrier_hub_torque = carrier_hub_torque
#
#     def passes_check(self, threshold):
#         return self._calculate_shear() < threshold and self._calculate_bending() < threshold
#
#     def get_name(self):
#         return "CarrierArm"
#
#     def get_fem_loads(self):
#         """
#         Returns bending moment and torsion for FEM.
#         """
#         m_bending = self._calculate_moment()
#         sigma_bending = self._calculate_sigma_bending(m_bending)
#         tau_torsion = (2 * self.carrier_hub_torque) / (
#                 math.pi * math.pow(self.planet.pitch_radius_mm , 3))  # simple solid shaft
#         return {
#             "M_bending": m_bending,
#             "sigma_bending": sigma_bending,
#             "tau_torsion": tau_torsion
#         }
#
#     def _calculate_shear(self):
#         # τ_arm = F_t,p / (w · t)
#         area_m2 = (self.width_mm ) * (self.thickness_mm )
#         return self.planet.effective_force / area_m2
#
#     def _calculate_bending(self):
#         m_arm = self._calculate_moment()
#         return self._calculate_sigma_bending(m_arm)
#
#     def _calculate_moment(self):
#         # M_arm = F_t,p · r_p
#         return self.planet.effective_force * self.planet.pitch_radius_mm 
#
#     def _calculate_sigma_bending(self, m_arm):
#         # I = w·t³ / 12
#         # c = t / 2
#         # σ_arm = M_arm · c / I
#         w_m = self.width_mm 
#         t_m = self.thickness_mm 
#
#         I = w_m * t_m**3 / 12
#         c = t_m / 2
#
#         return m_arm * c / I
#
#     def get_governing_stress(self):
#         return max(
#             self._calculate_shear(),
#             self._calculate_bending()
#         )


class CarrierHub(Component):
    def __init__(self, output_torque, radius_mm):
        self.output_torque = output_torque
        self.radius_mm = radius_mm

    def passes_check(self, threshold):
        return self._calculate_shear() < threshold

    def get_name(self):
        return "CarrierHub"

    def _calculate_shear(self):
        return (2 * self.output_torque) / (math.pi * math.pow(self.radius_mm , 3))  # simple solid shaft assumption

    def get_governing_stress(self):
        return self._calculate_shear()


class Stage:
    def __init__(self, index, *components):
        self.index = index
        self.components = components

    def display(self):
        print(f"Stage {self.index} results:")
        print(f"Component\tPass\tMargins\t\t\t\t\tFem\n")
        for component in self.components:
            component.display(MAX_SIGMA_ALLOWED_MEGA_PASCAL)
        print()

    def check_passed(self):
        return all([c.passes_check(MAX_SIGMA_ALLOWED_MEGA_PASCAL) for c in self.components])


current_input_torque = MOTOR_TORQUE_NEWTON_MM

for i in range(1, STAGES_COUNT + 1):

    # 1. Calculate effective tangential force for this stage
    effective_force = (current_input_torque / SUN_PITCH_RADIUS_MM) * LOAD_SHARING_FACTOR

    # 2. Create new component instances for this stage
    sun = Gear(SUN_TEETH_COUNT, SUN_FACE_WIDTH_MM, effective_force)
    planet = SecondaryGear(PLANET_TEETH_COUNT, PLANET_FACE_WIDTH_MM, effective_force)
    ring = Ring(RING_TEETH_COUNT, RING_FACE_WIDTH_MM, effective_force, RING_WALL_THICKNESS_MM)
    pin = Pin(planet, PIN_DIAMETER_MM, PIN_LENGTH_MM, PIN_FILLET_RADIUS_MM)

    # 3. Output torque for this stage
    stage_output_torque = current_input_torque * GEAR_RATIO

    # 4. Carrier components
    # carrier_arm = CarrierArm(planet, CARRIER_ARM_WIDTH_MM, CARRIER_ARM_THICKNESS_MM, stage_output_torque)
    carrier_hub = CarrierHub(stage_output_torque, CARRIER_HUB_RADIUS_MM)

    # 5. Create stage and store it
    stage = Stage(i, sun, planet, ring, pin, carrier_hub)

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
