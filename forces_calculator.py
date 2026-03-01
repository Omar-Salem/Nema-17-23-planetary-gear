import math
from abc import ABC, abstractmethod

# -------------------------
# GLOBAL PARAMETERS
# -------------------------
PLA_STRENGTH_MEGA_PASCAL = 45  # N/mm²
SAFETY_FACTOR = 3
EFFICIENCY = .8
GRAVITY_METER_SEC_SEC = 9.81
MAX_SIGMA_ALLOWED_PLA = PLA_STRENGTH_MEGA_PASCAL / SAFETY_FACTOR
LEWIS_CORRECTION_FACTOR = 1.2

# -------------------------
# GEAR SPECS
# -------------------------
GEAR_RATIO = 6

PLANETS_COUNT = 3
MODULE_MM = 1
PRESSURE_ANGLE_DEGREE = 20
PRESSURE_ANGLE_RADIANS = math.radians(PRESSURE_ANGLE_DEGREE)
LOAD_SHARING_FACTOR = 1
STAGES_COUNT = 2

SUN_TEETH_COUNT = 12
SUN_FACE_WIDTH_MM = 13
SUN_PITCH_RADIUS_MM = (MODULE_MM * SUN_TEETH_COUNT) / 2

RING_TEETH_COUNT = int((GEAR_RATIO - 1) * SUN_TEETH_COUNT)
RING_FACE_WIDTH_MM = 48

PLANET_TEETH_COUNT = int((GEAR_RATIO - 2) * SUN_TEETH_COUNT / 2)
PLANET_FACE_WIDTH_MM = 8

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
LOAD_WEIGHT_KG = 3
LOAD_LEVER_ARM_MM = 100

LOAD_TORQUE_N_MM = LOAD_WEIGHT_KG * GRAVITY_METER_SEC_SEC * LOAD_LEVER_ARM_MM


class Component:

    def __init__(self, threshold):
        self.threshold = threshold

    @abstractmethod
    def get_name(self):
        pass

    @abstractmethod
    def get_component_von_mises(self):
        pass

    @abstractmethod
    def get_fem_loads(self):
        pass

    def get_margin_data(self):
        sigma = self.get_component_von_mises()
        utilization = sigma / self.threshold
        margin_of_safety = (self.threshold / sigma) - 1 if sigma != 0 else float("inf")
        delta = self.threshold - sigma

        return sigma, utilization, margin_of_safety, delta

    def display(self):
        sigma, util, mos, _ = self.get_margin_data()
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
        return ", ".join([f"{k}: {v:g}" for k, v in fem_dict.items()])


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

    def __init__(self, teeth_count, face_width_mm, effective_force, name, threshold):
        super().__init__(threshold)
        self.teeth_count = teeth_count
        self.face_width_mm = face_width_mm
        self.pitch_radius_mm = (MODULE_MM * self.teeth_count) / 2
        self.effective_force = effective_force
        self.name = name

    def get_fem_loads(self):
        F_t = self.effective_force
        F_r = F_t * math.tan(math.radians(PRESSURE_ANGLE_DEGREE))

        return {
            "F_t (Tangential) N": F_t,
            "F_r (Radial) N": F_r
        }

    def passes_check(self):
        return self._calculate_bending_stress() < self.threshold

    def get_name(self):
        return self.name

    def _calculate_bending_stress(self):
        # σ = F / (b * m * y)
        lewis_y = self._get_lewis_form_factor(self.teeth_count)
        return self.effective_force / (self.face_width_mm * MODULE_MM * lewis_y * LEWIS_CORRECTION_FACTOR)

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

    def _calculate_shear_stress(self):
        # Area approximately face_width * module
        return self.effective_force / (self.face_width_mm * MODULE_MM)

    def get_component_von_mises(self):
        sigma_b = self._calculate_bending_stress()
        tau = self._calculate_shear_stress()
        return math.sqrt(math.pow(sigma_b, 2) + 3 * math.pow(tau, 2))


class Ring(Gear):
    def __init__(self, teeth_count, face_width_mm, tangential_force, radial_force, thickness, threshold):
        super().__init__(teeth_count, face_width_mm, tangential_force, "Ring", threshold)
        self.thickness = thickness
        self.radial_force = radial_force

    def passes_check(self):
        return self._calculate_bending_stress() < self.threshold and self._calculate_ovalization() < self.threshold

    def get_fem_loads(self):
        return {
            "F_t (Tangential) N": self.effective_force,
            "F_r (Radial) N": self.radial_force
        }

    def _calculate_bending_stress(self):
        y_ext = self._get_lewis_form_factor(self.teeth_count)
        y_internal = 1.3 * y_ext
        return self.effective_force / (self.face_width_mm * MODULE_MM * y_internal * LEWIS_CORRECTION_FACTOR)

    def _calculate_ovalization(self):
        # σ_ring ≈ (F_r,p · r_r) / (t_ring · b)
        return (self.radial_force * self.pitch_radius_mm) / (self.thickness * self.face_width_mm)

    def get_governing_stress(self):
        sigma_b = self._calculate_bending_stress()
        sigma_o = self._calculate_ovalization()
        # Total normal stress in this simplified approach
        return math.sqrt(math.pow(sigma_b + sigma_o, 2))


class Pin(Component):
    # Material properties
    YOUNG_MODULUS_PLA_N_MM = 2500  # MPa
    POISSONS_RATIO_PLA = 0.35

    # Derived shear moduli
    SHEAR_MODULUS_PLA = YOUNG_MODULUS_PLA_N_MM / (2 * (1 + POISSONS_RATIO_PLA))

    def __init__(self, force_N, diameter_mm, length_mm, fillet_radius_mm, threshold):
        super().__init__(threshold)
        self.F = force_N
        self.D = diameter_mm
        self.R = diameter_mm / 2
        self.L = length_mm
        self.fillet_radius = fillet_radius_mm

    # ---------------------------------------------------
    # Geometry helpers
    # ---------------------------------------------------
    def _area(self, r):
        return math.pi * math.pow(r, 2)

    def _inertia(self, r):
        return math.pi * math.pow(r, 4) / 4

    # ---------------------------------------------------
    # BENDING (Transformed section)
    # ---------------------------------------------------
    def _bending_stresses(self):
        M = self.F * self.L / 2  # cantilever end load (UDL approximation)
        I_outer = self._inertia(self.R)
        return self._sigma(M, I_outer)

    # ---------------------------------------------------
    # Stress concentration (PLA only)
    # ---------------------------------------------------
    def _kt(self):
        r_d = self.fillet_radius / self.D
        if r_d < 0.01: return 3.0
        if r_d < 0.05: return 2.2
        if r_d < 0.1:  return 1.7
        return 1.5

    def _von_mises(self):
        sigma = self._bending_stresses()
        tau = self._tau()

        sigma *= self._kt()

        return math.sqrt(
            math.pow(sigma, 2) + 3 * math.pow(tau, 2)
        )

    # ---------------------------------------------------
    # Deflection (true composite EI)
    # ---------------------------------------------------
    def _deflection(self):
        I_outer = self._inertia(self.R)
        EI = self.YOUNG_MODULUS_PLA_N_MM * I_outer
        return self.F * math.pow(self.L, 3) / (8 * EI)

    # ---------------------------------------------------
    # Reporting Methods
    # ---------------------------------------------------
    def get_name(self):
        return "Pin"

    def get_analytical_vm(self):
        return self._von_mises()

    def get_component_von_mises(self):
        return self._von_mises()

    def passes_check(self):
        return self._von_mises() <= self.threshold

    def get_fem_loads(self):
        """
        Calculates the Tangential and Radial components from the resultant Pin Force
        for easier entry into FEA 'Bearing Load' components.
        """
        # Derived from F_resultant = sqrt(Ft_total^2 + Fr_total^2)
        # where Fr = Ft * tan(Pressure_Angle)
        denom = math.sqrt(1 + math.pow(math.tan(PRESSURE_ANGLE_RADIANS), 2))
        f_tangential_applied = self.F / denom
        f_radial_applied = f_tangential_applied * math.tan(PRESSURE_ANGLE_RADIANS)

        return {
            "F_t (Tangential) N": round(f_tangential_applied, 2),
            "F_r (Radial) N": round(f_radial_applied, 2),
            "Deflection mm": round(self._deflection(), 4)
        }

    def _sigma(self, M, I_outer):
        sigma = M * self.R / I_outer
        return sigma

    def _tau(self):
        F = self.F
        A_total = self._area(self.R)
        tau = 4 * F / (3 * A_total)
        return tau


class SupportedPin(Pin):
    YOUNG_MODULUS_STEEL_N_MM = 200000  # MPa
    POISSONS_RATIO_STEEL = 0.30
    SIGMA_ALLOW_STEEL = 250

    SHEAR_MODULUS_STEEL = YOUNG_MODULUS_STEEL_N_MM / (2 * (1 + POISSONS_RATIO_STEEL))

    def __init__(self, force_N, diameter_mm, length_mm, fillet_radius_mm,
                 steel_bolt_diameter_mm, threshold):
        super().__init__(force_N, diameter_mm, length_mm, fillet_radius_mm, threshold)
        self.d_bolt = steel_bolt_diameter_mm
        self.r_bolt = steel_bolt_diameter_mm / 2

    def get_component_von_mises(self):
        return self._von_mises()

    def _deflection(self):
        I_outer = self._inertia(self.R)

        I_inner = self._inertia(self.r_bolt)
        EI = (
                self.YOUNG_MODULUS_PLA_N_MM * (I_outer - I_inner) +
                self.YOUNG_MODULUS_STEEL_N_MM * I_inner
        )

        return self.F * math.pow(self.L, 3) / (8 * EI)

    def _von_mises(self):
        sigma = self._bending_stresses()
        tau = self._tau()

        return math.sqrt(
            math.pow(sigma, 2) + 3 * math.pow(tau, 2)
        )

    def _tau(self):
        F = self.F
        A_total = self._area(self.R)

        A_steel = self._area(self.r_bolt)
        A_pla = A_total - A_steel

        share_steel = self.SHEAR_MODULUS_STEEL * A_steel
        share_pla = self.SHEAR_MODULUS_PLA * A_pla

        F_steel = F * share_steel / (share_steel + share_pla)

        tau_steel = 4 * F_steel / (3 * A_steel)

        return tau_steel

    def _sigma(self, M, I_outer):
        I_inner = self._inertia(self.r_bolt)
        n = self.YOUNG_MODULUS_STEEL_N_MM / self.YOUNG_MODULUS_PLA_N_MM

        I_trans = (I_outer - I_inner) + n * I_inner
        return n * M * self.r_bolt / I_trans

    def get_name(self):
        return "SupportedPin"


class CarrierHub(Component):
    def __init__(self,
                 torque_n_mm,
                 load_torque_n_mm,
                 shaft_radius_mm,
                 bolt_count,
                 bolt_circle_radius_mm,
                 insert_diameter_mm,
                 insert_embed_depth_mm, threshold):
        super().__init__(threshold)

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
        return math.sqrt(math.pow(sigma_b, 2) + 3 * math.pow(tau_t, 2))

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
    def get_component_von_mises(self):
        return max(
            self._shaft_von_mises(),
            self._insert_pullout()
        )

    def passes_check(self):
        return self.get_component_von_mises() < self.threshold

    # ------------------------
    # FEM OUTPUTS
    # ------------------------
    def get_fem_loads(self):
        return {
            "Input Torque N·mm": self.torque,
            "Load Torque (Bending) N·mm": self.load_torque_n_mm
        }


class Stage:
    def __init__(self, index, *components):
        self.index = index
        self.components = components

    def display(self):
        print(f"Stage {self.index} results:")
        print(f"Component\tPass\tVM MPa\t\tU\tMoS\t\t\t\tFem\n")
        for component in self.components:
            component.display()
        print()

    def check_passed(self):
        return all([c.passes_check() for c in self.components])


TOTAL_RATIO = math.pow(GEAR_RATIO, STAGES_COUNT)
TOTAL_EFFICIENCY = math.pow(EFFICIENCY, STAGES_COUNT)

required_motor_torque = LOAD_TORQUE_N_MM / (TOTAL_RATIO * TOTAL_EFFICIENCY)

current_input_torque = required_motor_torque

print(f"Required motor torque: {required_motor_torque:.2f} N·mm\n")

for i in range(1, STAGES_COUNT + 1):

    tangetial_force = (current_input_torque / (SUN_PITCH_RADIUS_MM * PLANETS_COUNT)) * LOAD_SHARING_FACTOR
    radial_force = tangetial_force * math.tan(PRESSURE_ANGLE_RADIANS)

    sun = Gear(SUN_TEETH_COUNT, SUN_FACE_WIDTH_MM, tangetial_force, "Sun", MAX_SIGMA_ALLOWED_PLA)
    planet = Gear(PLANET_TEETH_COUNT, PLANET_FACE_WIDTH_MM, tangetial_force, "Planet", MAX_SIGMA_ALLOWED_PLA)

    # Pass TOTAL radial force to ring
    ring_total_radial = PLANETS_COUNT * radial_force
    ring_total_tangential = PLANETS_COUNT * tangetial_force
    ring = Ring(RING_TEETH_COUNT, RING_FACE_WIDTH_MM, ring_total_tangential, ring_total_radial, RING_WALL_THICKNESS_MM,
                MAX_SIGMA_ALLOWED_PLA)

    # Total force transmitted through each planet pin
    pin_force = math.sqrt(
        math.pow(2 * tangetial_force, 2) +
        math.pow(2 * radial_force, 2)
    )
    pin = Pin(pin_force, PIN_DIAMETER_MM, PIN_LENGTH_MM, PIN_FILLET_RADIUS_MM, MAX_SIGMA_ALLOWED_PLA)

    # 3. Output torque for this stage
    stage_output_torque = current_input_torque * GEAR_RATIO * EFFICIENCY

    # 5. Create stage and store it
    stage = Stage(i, sun, planet, ring, pin)

    # 6. Display results
    stage.display()

    # 7. Stop if any component fails
    if not stage.check_passed():
        print(f"Stage {i} failed stress check ❌.")
        break
    else:
        if i == STAGES_COUNT:
            carrier_hub = CarrierHub(
                torque_n_mm=stage_output_torque,
                load_torque_n_mm=LOAD_TORQUE_N_MM,
                shaft_radius_mm=CARRIER_HUB_RADIUS_MM,
                bolt_count=CARRIER_HUB_BOLT_COUNT,
                bolt_circle_radius_mm=CARRIER_HUB_BOLT_CIRCLE_RADIUS_MM,
                insert_diameter_mm=HEAT_INSERT_DIAMETER_MM,
                insert_embed_depth_mm=HEAT_INSERT_EMBED_DEPTH_MM,
                threshold=MAX_SIGMA_ALLOWED_PLA
            )
            carrier_hub.display()
            if carrier_hub.passes_check():
                print(f"All {STAGES_COUNT} stages passed successfully! ✅")
            else:
                print(f"Carrier hub failed stress check ❌.")
        else:
            print(f"Stage {i} passed stress check ✅. Moving to next stage...\n")

    # 8. Prepare torque for next stage
    current_input_torque = stage_output_torque
