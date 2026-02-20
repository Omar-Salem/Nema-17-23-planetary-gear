import math
from abc import ABC, abstractmethod

# -------------------------
# GLOBAL PARAMETERS
# -------------------------
MATERIAL_STRENGTH_MEGA_PASCAL = 45  # N/mm²
SAFETY_FACTOR = 3
EFFICIENCY = .95
GRAVITY_METER_SEC_SEC = 9.81
MAX_SIGMA_ALLOWED_MEGA_PASCAL = MATERIAL_STRENGTH_MEGA_PASCAL / SAFETY_FACTOR

# -------------------------
# GEAR SPECS
# -------------------------
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

# -------------------------
# LOAD
# -------------------------
LOAD_WEIGHT_KG = 4
LOAD_LEVER_ARM_MM = 100

LOAD_TORQUE_N_MM = LOAD_WEIGHT_KG * GRAVITY_METER_SEC_SEC * LOAD_LEVER_ARM_MM


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
            f"{sigma :<15.2f} "
            f"{util:<5.2f} "
            f"{mos_str:<9} "
            f"{fem_output}"
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
    YOUNG_MODULUS_PLA_N_MM = 2500
    YOUNG_MODULUS_STEEL_N_MM = 200000

    def __init__(self, torque, diameter_mm, length_mm, fillet_radius_mm,
                 steel_bolt_diameter_mm=None):
        self.effective_force = torque
        self.diameter_mm = diameter_mm
        self.radius = diameter_mm / 2
        self.length_mm = length_mm
        self.fillet_radius_mm = fillet_radius_mm
        self.steel_bolt_diameter_mm = steel_bolt_diameter_mm

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
        Cantilever with UDL approximation:
        δ = FL³ / (8 E I)
        """

        I_pin = self._calculate_area_moment_of_inertia(self.diameter_mm)

        if self.steel_bolt_diameter_mm:
            I_bolt = self._calculate_area_moment_of_inertia(self.steel_bolt_diameter_mm)

            # Composite stiffness: E1I1 + E2I2
            EI_total = (
                    self.YOUNG_MODULUS_PLA_N_MM * I_pin +
                    self.YOUNG_MODULUS_STEEL_N_MM * I_bolt
            )
        else:
            EI_total = self.YOUNG_MODULUS_PLA_N_MM * I_pin

        return (self.effective_force * self.length_mm ** 3) / (8 * EI_total)

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
        Composite bending stress for concentric bonded materials.
        Returns maximum normal stress among materials.
        """

        # Geometry
        I_pin = self._calculate_area_moment_of_inertia(self.diameter_mm)
        c_pin = self.diameter_mm / 2

        if self.steel_bolt_diameter_mm:
            I_bolt = self._calculate_area_moment_of_inertia(self.steel_bolt_diameter_mm)
            c_bolt = self.steel_bolt_diameter_mm / 2

            # Composite bending stiffness
            EI_total = (
                    self.YOUNG_MODULUS_PLA_N_MM * I_pin +
                    self.YOUNG_MODULUS_STEEL_N_MM * I_bolt
            )

            # Curvature
            kappa = moment / EI_total

            # Stress in each material
            sigma_pin = self.YOUNG_MODULUS_PLA_N_MM * kappa * c_pin
            sigma_bolt = self.YOUNG_MODULUS_STEEL_N_MM * kappa * c_bolt

            return max(abs(sigma_pin), abs(sigma_bolt))

        else:
            # Single material (PLA only)
            EI_total = self.YOUNG_MODULUS_PLA_N_MM * I_pin
            kappa = moment / EI_total
            sigma_pin = self.YOUNG_MODULUS_PLA_N_MM * kappa * c_pin
            return abs(sigma_pin)

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

    def passes_check(self, stress_threshold, deflection_threshold=None):
        stress_ok = (
                self._calculate_von_mises() < stress_threshold and
                self._calculate_bearing() < stress_threshold
        )

        if deflection_threshold is not None:
            return stress_ok and self._calculate_deflection() < deflection_threshold

        return stress_ok

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
            "tau_shear": tau_pin,
            "deflection": self._calculate_deflection()
        }


class CarrierHub(Component):
    def __init__(self,
                 torque_n_mm,
                 load_torque_n_mm,
                 shaft_radius_mm,
                 bolt_count,
                 bolt_circle_radius_mm,
                 insert_diameter_mm,
                 insert_embed_depth_mm):

        # Applied loads
        self.torque = torque_n_mm
        self.load_torque_n_mm = load_torque_n_mm

        # Shaft geometry
        self.shaft_radius = shaft_radius_mm

        # Bolt pattern
        self.bolt_count = bolt_count
        self.bolt_circle_radius = bolt_circle_radius_mm

        # Insert geometry
        self.insert_diameter = insert_diameter_mm
        self.insert_embed_depth = insert_embed_depth_mm

    def get_name(self):
        return "CarrierHub"

    # ------------------------
    # SHAFT TORSION
    # τ = 2T / (π r³)
    # ------------------------
    def _shaft_torsion(self):
        return (2 * self.torque) / (
                math.pi * math.pow(self.shaft_radius, 3)
        )

    # ------------------------
    # SHAFT BENDING
    # σ = 4M / (π r³)
    # ------------------------
    def _shaft_bending(self):
        return (4 * self.load_torque_n_mm) / (
                math.pi * math.pow(self.shaft_radius, 3)
        )

    # ------------------------
    # COMBINED VON MISES (shaft)
    # ------------------------
    def _shaft_von_mises(self):
        sigma_b = self._shaft_bending()
        tau_t = self._shaft_torsion()
        return math.sqrt(sigma_b ** 2 + 3 * tau_t ** 2)

    # ------------------------
    # BOLT SHEAR FROM TORQUE
    # F = T / (n r)
    # ------------------------
    def _bolt_shear_force(self):
        if self.bolt_count == 0:
            return 0
        return self.torque / (
                self.bolt_count * self.bolt_circle_radius
        )

    # ------------------------
    # BOLT TENSION FROM BENDING
    # Worst case linear distribution
    # F = M / (n r)
    # ------------------------
    def _bolt_tension_from_bending(self):
        if self.bolt_count == 0:
            return 0
        return self.load_torque_n_mm / (
                self.bolt_count * self.bolt_circle_radius
        )

    # ------------------------
    # INSERT PULL-OUT STRESS
    # σ = F / (π d h)
    # ------------------------
    def _insert_pullout(self):
        tension = self._bolt_tension_from_bending()

        shear_area = (
                math.pi *
                self.insert_diameter *
                self.insert_embed_depth
        )

        if shear_area == 0:
            return 0

        return tension / shear_area

    # ------------------------
    # GOVERNING STRESS
    # ------------------------
    def get_governing_stress(self):
        return max(
            self._shaft_von_mises(),
            self._insert_pullout()
        )

    def passes_check(self, threshold):
        return self.get_governing_stress() < threshold

    # ------------------------
    # FEM OUTPUTS
    # ------------------------
    def get_fem_loads(self):
        return {
            "shaft_vm": self._shaft_von_mises(),
            "shaft_bending": self._shaft_bending(),
            "shaft_torsion": self._shaft_torsion(),
            "bolt_shear_force": self._bolt_shear_force(),
            "bolt_tension": self._bolt_tension_from_bending(),
            "insert_pullout": self._insert_pullout()
        }


class Stage:
    def __init__(self, index, *components):
        self.index = index
        self.components = components

    def display(self):
        print(f"Stage {self.index} results:")
        print(f"Component\tPass\tForce σ MPa\tU\tMoS\t\t\t\t\tFem\n")
        for component in self.components:
            component.display(MAX_SIGMA_ALLOWED_MEGA_PASCAL)
        print()

    def check_passed(self):
        return all([c.passes_check(MAX_SIGMA_ALLOWED_MEGA_PASCAL) for c in self.components])


TOTAL_RATIO = math.pow(GEAR_RATIO, STAGES_COUNT)
TOTAL_EFFICIENCY = math.pow(EFFICIENCY, STAGES_COUNT)

required_motor_torque = LOAD_TORQUE_N_MM / (TOTAL_RATIO * TOTAL_EFFICIENCY)

current_input_torque = required_motor_torque

for i in range(1, STAGES_COUNT + 1):

    # 1. Calculate effective tangential force for this stage
    tangetial_sun_force = (current_input_torque / (SUN_PITCH_RADIUS_MM * PLANETS_COUNT)) * LOAD_SHARING_FACTOR

    # 2. Create new component instances for this stage
    sun = Gear(SUN_TEETH_COUNT, SUN_FACE_WIDTH_MM, tangetial_sun_force, "Sun")
    planet = Gear(PLANET_TEETH_COUNT, PLANET_FACE_WIDTH_MM, tangetial_sun_force, "Planet")
    ring = Ring(RING_TEETH_COUNT, RING_FACE_WIDTH_MM, tangetial_sun_force, RING_WALL_THICKNESS_MM)
    pin = Pin(2 * tangetial_sun_force, PIN_DIAMETER_MM, PIN_LENGTH_MM, PIN_FILLET_RADIUS_MM)

    # 3. Output torque for this stage
    stage_output_torque = current_input_torque * GEAR_RATIO * EFFICIENCY

    carrier_hub = CarrierHub(
        torque_n_mm=stage_output_torque,
        load_torque_n_mm=LOAD_TORQUE_N_MM,
        shaft_radius_mm=CARRIER_HUB_RADIUS_MM,
        bolt_count=CARRIER_HUB_BOLT_COUNT,
        bolt_circle_radius_mm=CARRIER_HUB_BOLT_CIRCLE_RADIUS_MM,
        insert_diameter_mm=HEAT_INSERT_DIAMETER_MM,
        insert_embed_depth_mm=HEAT_INSERT_EMBED_DEPTH_MM
    )

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
