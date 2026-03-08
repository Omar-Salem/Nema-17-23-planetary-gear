import math
from abc import ABC, abstractmethod

# -------------------------
# ASSUMPTIONS
# -------------------------
LEWIS_CORRECTION_FACTOR = 1.2
EFFICIENCY = 0.8
SAFETY_FACTOR = 3
LOAD_SHARING_FACTOR = 1

# -------------------------
# GLOBAL PARAMETERS
# -------------------------
GRAVITY_METER_SEC_SEC = 9.81
PLA_STRENGTH_MEGA_PASCAL = 45  # N/mm²
MAX_SIGMA_ALLOWED_PLA = PLA_STRENGTH_MEGA_PASCAL / SAFETY_FACTOR

STEEL_STRENGTH_MEGA_PASCAL = 1080
MAX_SIGMA_ALLOWED_STEEL = STEEL_STRENGTH_MEGA_PASCAL / SAFETY_FACTOR

# -------------------------
# GEAR SPECS
# -------------------------
GEAR_RATIO = 6

PLANETS_COUNT = 3
MODULE_MM = 1
PRESSURE_ANGLE_DEGREE = 20
PRESSURE_ANGLE_RADIANS = math.radians(PRESSURE_ANGLE_DEGREE)
STAGES_COUNT = 2

# -------------------------
# SUN
# -------------------------
SUN_TEETH_COUNT = 12
SUN_FACE_WIDTH_MM = 13
SUN_PITCH_RADIUS_MM = (MODULE_MM * SUN_TEETH_COUNT) / 2

# -------------------------
# PLANET
# -------------------------
PLANET_TEETH_COUNT = int((GEAR_RATIO - 2) * SUN_TEETH_COUNT / 2)
PLANET_FACE_WIDTH_MM = 8

# -------------------------
# PIN
# -------------------------
PIN_DIAMETER_MM = 5.27
PIN_LENGTH_MM = 5
PIN_FILLET_RADIUS_MM = 0.5

M3_BOLT_DIAMETER_MM = 3.0

# -------------------------
# RING
# -------------------------
RING_TEETH_COUNT = int((GEAR_RATIO - 1) * SUN_TEETH_COUNT)
RING_FACE_WIDTH_MM = 48
RING_WALL_THICKNESS_MM = 8

# -------------------------
# CARRIER
# -------------------------
CARRIER_HUB_RADIUS_MM = 35.1 / 2
CARRIER_HUB_BOLT_COUNT = 8
CARRIER_HUB_BOLT_CIRCLE_RADIUS_MM = 13.2

# -------------------------
# HEAT_INSERT
# -------------------------
HEAT_INSERT_DIAMETER_MM = 4.2
HEAT_INSERT_EMBED_DEPTH_MM = 5

# -------------------------
# LOAD
# -------------------------
LOAD_WEIGHT_KG = 3
LOAD_LEVER_ARM_MM = 100
LOAD_TORQUE_N_MM = LOAD_WEIGHT_KG * GRAVITY_METER_SEC_SEC * LOAD_LEVER_ARM_MM


class Component(ABC):
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

        mos_str = " >9999" if mos > 9999 else f"{mos:7.2f}"

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
        return {"F_t (Tangential) N": F_t, "F_r (Radial) N": F_r}

    def passes_check(self):
        return self._calculate_bending_stress() < self.threshold

    def get_name(self):
        return self.name

    def _calculate_bending_stress(self):
        lewis_y = self._get_lewis_form_factor(self.teeth_count)
        return self.effective_force / (self.face_width_mm * MODULE_MM * lewis_y * LEWIS_CORRECTION_FACTOR)

    def _get_lewis_form_factor(self, teeth):
        if teeth in self.EXTERNAL_LEWIS_20_TABLE:
            return self.EXTERNAL_LEWIS_20_TABLE[teeth]
        keys = sorted(self.EXTERNAL_LEWIS_20_TABLE.keys())
        for i in range(len(keys) - 1):
            low, high = keys[i], keys[i + 1]
            if low < teeth < high:
                y_low = self.EXTERNAL_LEWIS_20_TABLE[low]
                y_high = self.EXTERNAL_LEWIS_20_TABLE[high]
                frac = (teeth - low) / (high - low)
                return y_low + (y_high - y_low) * frac
        return self.EXTERNAL_LEWIS_20_TABLE[keys[-1] if teeth > keys[-1] else keys[0]]

    def _calculate_shear_stress(self):
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
        return {"F_t (Tangential) N": self.effective_force, "F_r (Radial) N": self.radial_force}

    def _calculate_bending_stress(self):
        y_ext = self._get_lewis_form_factor(self.teeth_count)
        y_internal = 1.3 * y_ext
        return self.effective_force / (self.face_width_mm * MODULE_MM * y_internal * LEWIS_CORRECTION_FACTOR)

    def _calculate_ovalization(self):
        return (self.radial_force * self.pitch_radius_mm) / (self.thickness * self.face_width_mm)

    def get_governing_stress(self):
        sigma_b = self._calculate_bending_stress()
        sigma_o = self._calculate_ovalization()
        return math.sqrt(math.pow(sigma_b + sigma_o, 2))


class Pin(Component):
    YOUNG_MODULUS_PLA_N_MM = 2500
    POISSONS_RATIO_PLA = 0.35
    SHEAR_MODULUS_PLA = YOUNG_MODULUS_PLA_N_MM / (2 * (1 + POISSONS_RATIO_PLA))

    def __init__(self, force_N, diameter_mm, length_mm, fillet_radius_mm, threshold):
        super().__init__(threshold)
        self.F = force_N
        self.D = diameter_mm
        self.R = diameter_mm / 2
        self.L = length_mm
        self.fillet_radius = fillet_radius_mm

    def _area(self, r):
        return math.pi * math.pow(r, 2)

    def _inertia(self, r):
        return math.pi * math.pow(r, 4) / 4

    def _bending_stresses(self):
        M = self.F * self.L / 2
        I_outer = self._inertia(self.R)
        return self._sigma(M, I_outer)

    def _kt(self):
        r_d = self.fillet_radius / self.D
        if r_d < 0.01: return 3.0
        if r_d < 0.05: return 2.2
        if r_d < 0.1:  return 1.7
        return 1.5

    def _von_mises(self):
        sigma = self._bending_stresses() * self._kt()
        tau = self._tau()
        return math.sqrt(math.pow(sigma, 2) + 3 * math.pow(tau, 2))

    def _deflection(self):
        I = self._inertia(self.R)
        EI = self.YOUNG_MODULUS_PLA_N_MM * I
        return self.F * math.pow(self.L, 3) / (8 * EI)

    def get_name(self):
        return "Pin"

    def get_analytical_vm(self):
        return self._von_mises()

    def get_component_von_mises(self):
        return self._von_mises()

    def passes_check(self):
        return self._von_mises() <= self.threshold

    def get_fem_loads(self):
        denom = math.sqrt(1 + math.pow(math.tan(PRESSURE_ANGLE_RADIANS), 2))
        f_tangential_applied = self.F / denom
        f_radial_applied = f_tangential_applied * math.tan(PRESSURE_ANGLE_RADIANS)
        return {
            "F_t (Tangential) N": round(f_tangential_applied, 2),
            "F_r (Radial) N": round(f_radial_applied, 2),
            "Deflection mm": round(self._deflection(), 4)
        }

    def _sigma(self, M, I):
        return M * self.R / I

    def _tau(self):
        area = self._area(self.R)
        return 4 * self.F / (3 * area)


class SupportedPin(Pin):
    YOUNG_MODULUS_STEEL_N_MM = 200000
    POISSONS_RATIO_STEEL = 0.30
    SHEAR_MODULUS_STEEL = YOUNG_MODULUS_STEEL_N_MM / (2 * (1 + POISSONS_RATIO_STEEL))

    def __init__(self, force_N, diameter_mm, length_mm, fillet_radius_mm, threshold, steel_bolt_diameter_mm):
        super().__init__(force_N, diameter_mm, length_mm, fillet_radius_mm, threshold)
        self.d_bolt = steel_bolt_diameter_mm
        self.r_bolt = steel_bolt_diameter_mm / 2

    def get_component_von_mises(self):
        return self._von_mises()

    def _deflection(self):
        I_outer = self._inertia(self.R)
        I_inner = self._inertia(self.r_bolt)
        EI = (self.YOUNG_MODULUS_PLA_N_MM * (I_outer - I_inner) + self.YOUNG_MODULUS_STEEL_N_MM * I_inner)
        return self.F * math.pow(self.L, 3) / (8 * EI)

    def _von_mises(self):
        sigma = self._bending_stresses()
        tau = self._tau()
        return math.sqrt(math.pow(sigma, 2) + 3 * math.pow(tau, 2))

    def _tau(self):
        A_total = self._area(self.R)
        A_steel = self._area(self.r_bolt)
        A_pla = A_total - A_steel

        share_steel = self.SHEAR_MODULUS_STEEL * A_steel
        share_pla = self.SHEAR_MODULUS_PLA * A_pla

        F_steel = self.F * share_steel / (share_steel + share_pla)
        return 4 * F_steel / (3 * A_steel)

    def _sigma(self, M, I_outer):
        I_inner = self._inertia(self.r_bolt)
        n = self.YOUNG_MODULUS_STEEL_N_MM / self.YOUNG_MODULUS_PLA_N_MM
        I_trans = (I_outer - I_inner) + n * I_inner
        return n * M * self.r_bolt / I_trans

    def get_name(self):
        return "SupportedPin"


class CarrierHub(Component):
    def __init__(self, torque_n_mm, load_torque_n_mm, shaft_radius_mm, bolt_count, bolt_circle_radius_mm, insert_diameter_mm, insert_embed_depth_mm, threshold):
        super().__init__(threshold)
        self.torque = torque_n_mm
        self.load_torque_n_mm = load_torque_n_mm
        self.shaft_radius = shaft_radius_mm
        self.bolt_count = bolt_count
        self.bolt_circle_radius = bolt_circle_radius_mm
        self.insert_diameter = insert_diameter_mm
        self.insert_embed_depth = insert_embed_depth_mm

    def get_name(self):
        return "CarrierHub"

    def _shaft_torsion(self):
        return (2 * self.torque) / (math.pi * math.pow(self.shaft_radius, 3))

    def _shaft_bending(self):
        return (4 * self.load_torque_n_mm) / (math.pi * math.pow(self.shaft_radius, 3))

    def _shaft_von_mises(self):
        sigma_b = self._shaft_bending()
        tau_t = self._shaft_torsion()
        return math.sqrt(math.pow(sigma_b, 2) + 3 * math.pow(tau_t, 2))

    def _bolt_shear_force(self):
        if self.bolt_count == 0: return 0
        return self.torque / (self.bolt_count * self.bolt_circle_radius)

    def _bolt_tension_from_bending(self):
        if self.bolt_count == 0: return 0
        return self.load_torque_n_mm / (self.bolt_count * self.bolt_circle_radius)

    def _insert_pullout(self):
        tension = self._bolt_tension_from_bending()
        shear_area = math.pi * self.insert_diameter * self.insert_embed_depth
        if shear_area == 0: return 0
        return tension / shear_area

    def get_component_von_mises(self):
        return max(self._shaft_von_mises(), self._insert_pullout())

    def passes_check(self):
        return self.get_component_von_mises() < self.threshold

    def get_fem_loads(self):
        return {"Input Torque N·mm": self.torque, "Load Torque (Bending) N·mm": self.load_torque_n_mm}


class Stage:
    def __init__(self, index, *components):
        self.index = index
        self.components = components
        
    def get_stage_utilization(self):
        return max([c.get_margin_data()[1] for c in self.components])
    
    def display(self):
        print(f"Stage {self.index} results:")
        print("Component\tPass\tVM MPa\t\tU\tMoS\t\t\t\tFem\n")
        for component in self.components:
            component.display()
        print()

    def check_passed(self):
        return all([c.passes_check() for c in self.components])


# ---------------------------------------------------------
# NEW ENCAPSULATED LOGIC FOR LINEAR SCALING
# ---------------------------------------------------------
def evaluate_system_utilization(load_weight_kg, efficiency, display_results=False):

    load_torque = load_weight_kg * GRAVITY_METER_SEC_SEC * LOAD_LEVER_ARM_MM
    total_ratio = math.pow(GEAR_RATIO, STAGES_COUNT)
    total_efficiency = math.pow(efficiency, STAGES_COUNT)

    current_input_torque = load_torque / (total_ratio * total_efficiency)

    for i in range(1, STAGES_COUNT + 1):
        tangential_force = (current_input_torque / (SUN_PITCH_RADIUS_MM * PLANETS_COUNT)) * LOAD_SHARING_FACTOR
        radial_force = tangential_force * math.tan(PRESSURE_ANGLE_RADIANS)

        sun = Gear(SUN_TEETH_COUNT, SUN_FACE_WIDTH_MM, tangential_force, "Sun", MAX_SIGMA_ALLOWED_PLA)
        planet = Gear(PLANET_TEETH_COUNT, PLANET_FACE_WIDTH_MM, tangential_force, "Planet", MAX_SIGMA_ALLOWED_PLA)

        ring_total_radial = PLANETS_COUNT * radial_force
        ring_total_tangential = PLANETS_COUNT * tangential_force
        ring = Ring(RING_TEETH_COUNT, RING_FACE_WIDTH_MM, ring_total_tangential, ring_total_radial, RING_WALL_THICKNESS_MM, MAX_SIGMA_ALLOWED_PLA)

        pin_force = math.sqrt(math.pow(2 * tangential_force, 2) + math.pow(2 * radial_force, 2))
        pin = Pin(pin_force, PIN_DIAMETER_MM, PIN_LENGTH_MM, PIN_FILLET_RADIUS_MM, MAX_SIGMA_ALLOWED_PLA)
        pin = Pin(pin_force, PIN_DIAMETER_MM, PIN_LENGTH_MM, PIN_FILLET_RADIUS_MM,
                      MAX_SIGMA_ALLOWED_PLA) if i < STAGES_COUNT else SupportedPin(
                pin_force, PIN_DIAMETER_MM, PIN_LENGTH_MM, PIN_FILLET_RADIUS_MM, MAX_SIGMA_ALLOWED_STEEL, M3_BOLT_DIAMETER_MM)

        carrier_hub = CarrierHub(
            torque_n_mm=current_input_torque, # This is the final output torque matching the payload
            load_torque_n_mm=load_torque,
            shaft_radius_mm=CARRIER_HUB_RADIUS_MM,
            bolt_count=CARRIER_HUB_BOLT_COUNT,
            bolt_circle_radius_mm=CARRIER_HUB_BOLT_CIRCLE_RADIUS_MM,
            insert_diameter_mm=HEAT_INSERT_DIAMETER_MM,
            insert_embed_depth_mm=HEAT_INSERT_EMBED_DEPTH_MM,
            threshold=MAX_SIGMA_ALLOWED_PLA
        )
        stage = Stage(i, sun, planet, ring, pin, carrier_hub)

        if display_results:
            stage.display()
            if not stage.check_passed():
                print(f"Stage {i} failed stress check ❌.\n")
            else:
                print(f"Stage {i} passed stress check ✅.\n")

        stage_output_torque = current_input_torque * GEAR_RATIO * efficiency
        current_input_torque = stage_output_torque

    return stage.get_stage_utilization()


def find_max_safe_load(test_efficiency):
    """
    Uses linearity to find the maximum safe load in O(1) time.
    Calculates the utilization for a 1kg dummy load, then scales up.
    """
    dummy_load_kg = 1.0
    max_util = evaluate_system_utilization(dummy_load_kg, test_efficiency, display_results=False)

    # Scale load to reach exactly 1.0 utilization
    max_safe_load_kg = dummy_load_kg / max_util
    max_safe_torque = max_safe_load_kg * GRAVITY_METER_SEC_SEC * LOAD_LEVER_ARM_MM

    return max_safe_load_kg, max_safe_torque


# ---------------------------------------------------------
# EXECUTION
# ---------------------------------------------------------
if __name__ == "__main__":
    print("-" * 50)
    print("1. SYSTEM CHECK")
    print("-" * 50)
    # Replicates your original script's exact output block
    evaluate_system_utilization(LOAD_WEIGHT_KG, EFFICIENCY, display_results=True)

    print("-" * 50)
    print("2. EFFICIENCY SWEEP: MAXIMUM SAFE LOAD CAPACITY")
    print("-" * 50)
    for eff in [EFFICIENCY, 0.90]:
        max_kg, max_torque = find_max_safe_load(test_efficiency=eff)
        print(f"Efficiency {eff*100:.0f}% | Max Safe Load: {max_kg:5.2f} kg ({max_torque:7.0f} N·mm)")