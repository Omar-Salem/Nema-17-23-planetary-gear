import math
from abc import ABC, abstractmethod

MOTOR_TORQUE_NEWTON_MM = 17
MATERIAL_STRENGTH_MEGA_PASCAL = 38  # N/mm²
SAFETY_FACTOR = 3
EFFICIENCY = .95

GEAR_RATIO = 6

PLANETS_COUNT = 3
MODULE_MM = 1
PRESSURE_ANGLE_DEGREE = 20
LOAD_SHARING_FACTOR = 1
STAGES_COUNT = 2

SUN_TEETH_COUNT = 12
SUN_FACE_WIDTH_MM = 13
SUN_PITCH_RADIUS_MM = (MODULE_MM * SUN_TEETH_COUNT) / 2

RING_TEETH_COUNT = int((GEAR_RATIO - 1) * SUN_TEETH_COUNT)
RING_FACE_WIDTH_MM = 48

PLANET_TEETH_COUNT = int((GEAR_RATIO - 2) * SUN_TEETH_COUNT / 2)
PLANET_FACE_WIDTH_MM = 8

CARRIER_ARM_WIDTH_MM = 6.3
CARRIER_ARM_THICKNESS_MM = 8

PIN_DISTANCE_FROM_CENTER_MM = 18
PIN_DIAMETER_MM = 5.27
PIN_LENGTH_MM = 5
PIN_FILLET_RADIUS_MM = .5

RING_WALL_THICKNESS_MM = 8

CARRIER_HUB_RADIUS_MM = 35.1 / 2
CARRIER_HUB_BOLT_COUNT = 8
CARRIER_HUB_BOLT_CIRCLE_RADIUS_MM = 13.2
HEAT_INSERT_DIAMETER_MM = 4.2
HEAT_INSERT_EMBED_DEPTH_MM = 5

AXIAL_LOAD_N = 0
BENDING_FORCE_ACC_SF = 2  # Account for dynamic effects
BENDING_FORCE_N = 40  # ~4 Kg
MAX_BENDING_FORCE_N = BENDING_FORCE_N * BENDING_FORCE_ACC_SF
BENDING_LEVER_ARM_MM = 100

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

    def __init__(self, teeth_count, face_width_mm, effective_force, name):
        self.teeth_count = teeth_count
        self.face_width_mm = face_width_mm
        self.pitch_radius_mm = (MODULE_MM * self.teeth_count) / 2
        self.effective_force = effective_force
        self.name = name
        
    def passes_check(self, threshold):
        return self._calculate_bending_stress() < threshold

    def get_name(self):
        return self.name

    def _calculate_bending_stress(self):
        # σ = F / (b * m * y)
        lewis_y = self._get_lewis_form_factor(self.teeth_count)
        return self.effective_force / (self.face_width_mm * MODULE_MM * lewis_y)

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


class Ring(Gear):
    def __init__(self, teeth_count, face_width_mm, effective_force, thickness):
        super().__init__(teeth_count, face_width_mm, effective_force, "Ring")
        self.thickness = thickness
        self.radial_force = self.effective_force * math.tan(math.radians(PRESSURE_ANGLE_DEGREE))

    def passes_check(self, threshold):
        return self._calculate_bending_stress() < threshold and self._calculate_ovalization() < threshold

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
        return self.effective_force / (self.face_width_mm * MODULE_MM * y_internal)

    def _calculate_ovalization(self):
        # σ_ring ≈ (F_r,p · r_r) / (t_ring · b)
        return (self.radial_force * self.pitch_radius_mm) / (self.thickness * self.face_width_mm)

    def get_governing_stress(self):
        return max(
            self._calculate_bending_stress(),
            self._calculate_ovalization()
        )


class Pin(Component):
    def __init__(self, torque, diameter_mm, length_mm, fillet_radius_mm,
                 bolt_diameter_mm=None):
        self.effective_force = torque 
        self.diameter_mm = diameter_mm
        self.radius = diameter_mm / 2
        self.length_mm = length_mm
        self.fillet_radius_mm = fillet_radius_mm
        self.bolt_diameter_mm = bolt_diameter_mm

    def _calculate_bending(self):
        """
        4FL/3πr³ 
        """
        return 4 * self.effective_force * self.length_mm / (3 * math.pi * math.pow(self.radius, 3))

    def _calculate_shear(self):
        area = math.pi * math.pow(self.radius, 2)
        peak_stress = (4 * self.effective_force) / (3 * area)  # (Parabolic Shear Stress Theory)
        average_stress = self.effective_force / area
        return peak_stress

    def _calculate_deflection(self):
        """
        FL³/8EI
        """
        E = 2500  # Young's modulus for PLA in N/mm²
        I = self._calculate_area_moment_of_inertia(self.diameter_mm)
        return (self.effective_force * math.pow(self.length_mm, 3)) / (8 * E * I)

    def _calculate_moment(self):
        """
        Calculates max moment for a Cantilever with Uniformly Distributed Load (UDL).
        Formula: FL/2
        """
        return (self.effective_force * self.length_mm) / 2

    def _calculate_area_moment_of_inertia(self, diameter_mm):
        """
        I = π * r⁴ / 4 for solid circular cross-section
        """
        radius = diameter_mm / 2
        return (math.pi * math.pow(radius, 4)) / 4

    def _calculate_sigma(self, moment):
        """
        Bending stress using combined inertia of pin + bolt if present.
        """
        I_pin = self._calculate_area_moment_of_inertia(self.diameter_mm)

        if self.bolt_diameter_mm:
            I_bolt = self._calculate_area_moment_of_inertia(self.bolt_diameter_mm)
            I_eff = I_pin + I_bolt
            c_max = max(self.diameter_mm, self.bolt_diameter_mm) / 2
        else:
            I_eff = I_pin
            c_max = self.diameter_mm / 2

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
        area_proj = (self.diameter_mm) * (self.length_mm)
        return self.effective_force / area_proj

    def get_governing_stress(self):
        return max(
            self._calculate_von_mises(),
            self._calculate_bearing()
        )

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


class CarrierHub(Component):
    def __init__(self, torque,
                 shaft_radius_mm,
                 bolt_count,
                 bolt_circle_radius_mm,
                 insert_diameter_mm,
                 insert_embed_depth_mm,
                 axial_load_n=0,
                 bending_force_n=0,
                 bending_lever_arm_mm=0):

        self.torque = torque
        self.shaft_radius_mm = shaft_radius_mm

        self.bolt_count = bolt_count
        self.bolt_circle_radius_mm = bolt_circle_radius_mm

        self.insert_diameter_mm = insert_diameter_mm
        self.insert_embed_depth_mm = insert_embed_depth_mm

        self.axial_load_n = axial_load_n
        self.bending_force_n = bending_force_n
        self.bending_lever_arm_mm = bending_lever_arm_mm

    def get_name(self):
        return "CarrierHub"

    # ------------------------
    # SHAFT TORSION
    # ------------------------
    def _calculate_shaft_shear(self):
        return (2 * self.torque) / (
                math.pi * math.pow(self.shaft_radius_mm, 3)
        )

    # ------------------------
    # BOLT SHEAR FROM TORQUE
    # ------------------------
    def _calculate_bolt_shear(self):
        if self.bolt_count == 0:
            return 0
        return self.torque / (
                self.bolt_count * self.bolt_circle_radius_mm
        )

    # ------------------------
    # BENDING MOMENT EFFECT
    # ------------------------
    def _calculate_bolt_tension_from_bending(self):
        if self.bending_force_n == 0:
            return 0

        moment = self.bending_force_n * self.bending_lever_arm_mm

        # worst-case bolt (simplified full moment on one side)
        return moment / (
                self.bolt_circle_radius_mm * self.bolt_count
        )

    # ------------------------
    # INSERT PULL-OUT STRESS
    # ------------------------
    def _calculate_insert_pullout_stress(self):
        tension = (
                self.axial_load_n / self.bolt_count
                + self._calculate_bolt_tension_from_bending()
        )

        shear_area = (
                math.pi *
                self.insert_diameter_mm *
                self.insert_embed_depth_mm
        )

        if shear_area == 0:
            return 0

        return tension / shear_area

    # ------------------------
    # GOVERNING STRESS
    # ------------------------
    def get_governing_stress(self):
        return max(
            self._calculate_shaft_shear(),
            self._calculate_insert_pullout_stress()
        )

    def get_fem_loads(self):
        return {
            "shaft_tau": self._calculate_shaft_shear(),
            "bolt_shear": self._calculate_bolt_shear(),
            "insert_pullout_sigma": self._calculate_insert_pullout_stress()
        }

    def passes_check(self, threshold):
        return self.get_governing_stress() < threshold


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
    tangetial_sun_force = (current_input_torque / (SUN_PITCH_RADIUS_MM* PLANETS_COUNT)) * LOAD_SHARING_FACTOR

    # 2. Create new component instances for this stage
    sun = Gear(SUN_TEETH_COUNT, SUN_FACE_WIDTH_MM, tangetial_sun_force,"Sun")
    planet = Gear(PLANET_TEETH_COUNT, PLANET_FACE_WIDTH_MM, tangetial_sun_force,"Planet")
    ring = Ring(RING_TEETH_COUNT, RING_FACE_WIDTH_MM, tangetial_sun_force, RING_WALL_THICKNESS_MM)
    pin = Pin(2*tangetial_sun_force, PIN_DIAMETER_MM, PIN_LENGTH_MM, PIN_FILLET_RADIUS_MM)

    # 3. Output torque for this stage
    stage_output_torque = current_input_torque * GEAR_RATIO * EFFICIENCY

    carrier_hub = CarrierHub(
        stage_output_torque,
        CARRIER_HUB_RADIUS_MM,
        CARRIER_HUB_BOLT_COUNT,
        CARRIER_HUB_BOLT_CIRCLE_RADIUS_MM,
        HEAT_INSERT_DIAMETER_MM,
        HEAT_INSERT_EMBED_DEPTH_MM,
        AXIAL_LOAD_N,
        MAX_BENDING_FORCE_N,
        BENDING_LEVER_ARM_MM)

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
