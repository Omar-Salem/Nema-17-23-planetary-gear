import math
from abc import ABC, abstractmethod
from typing import Dict, List, Tuple

# -------------------------
# ASSUMPTIONS
# -------------------------
LEWIS_CORRECTION_FACTOR = 1.2
GEAR_EFFICIENCY = 0.8
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

# -------------------------
# MOTOR
# -------------------------
MOTOR_TORQUE_N_MM = 340 


class Component(ABC):
    def __init__(self, name: str, threshold: float) -> None:
        self.name = name
        self.threshold = threshold

    def get_name(self) -> str:
        return self.name

    @abstractmethod
    def get_component_von_mises(self) -> float:
        pass

    @abstractmethod
    def get_fem_loads(self) -> Dict[str, float]:
        pass

    def get_margin_data(self) -> Tuple[float, float, float, float]:
        sigma = self.get_component_von_mises()
        utilization = sigma / self.threshold
        margin_of_safety = (self.threshold / sigma) - 1 if sigma != 0 else float("inf")
        delta = self.threshold - sigma
        return sigma, utilization, margin_of_safety, delta

    def display(self) -> None:
        sigma, util, mos, _ = self.get_margin_data()
        check = "✅" if util < 1 else "❌"
        mos_str = " >9999" if mos > 9999 else f"{mos:7.2f}"
        print(
            f"{self.get_name():<15}"
            f"{check:<7}"
            f"{sigma :<15.2f} "
            f"{util:<5.2f} "
            f"{mos_str:<9} "
            f"{self._format_fem()}"
        )

    def _format_fem(self) -> str:
        fem_dict = self.get_fem_loads()
        if not fem_dict:
            return "-"
        return ", ".join([f"{k}: {v:g}" for k, v in fem_dict.items()])


class Gear(Component):
    EXTERNAL_LEWIS_20_TABLE: Dict[int, float] = {
        10: 0.201, 11: 0.226, 12: 0.245, 13: 0.264, 14: 0.276,
        15: 0.289, 16: 0.295, 17: 0.302, 18: 0.308, 19: 0.314,
        20: 0.320, 22: 0.330, 24: 0.337, 26: 0.344, 28: 0.352,
        30: 0.358, 32: 0.364, 34: 0.370, 36: 0.377, 38: 0.383,
        40: 0.389, 45: 0.399, 50: 0.408, 55: 0.415, 60: 0.421,
        65: 0.425, 70: 0.429, 75: 0.433, 80: 0.436, 90: 0.442,
        100: 0.446, 150: 0.458, 200: 0.463, 300: 0.471
    }

    def __init__(self, teeth_count: int, face_width_mm: float, effective_force: float, name: str,
                 threshold: float) -> None:
        super().__init__(name, threshold)
        self.teeth_count = teeth_count
        self.face_width_mm = face_width_mm
        self.pitch_radius_mm = (MODULE_MM * self.teeth_count) / 2
        self.effective_force = effective_force
        self.name = name

    def get_fem_loads(self) -> Dict[str, float]:
        F_t = self.effective_force
        F_r = F_t * math.tan(math.radians(PRESSURE_ANGLE_DEGREE))
        return {"F_t (Tangential) N": F_t, "F_r (Radial) N": F_r}

    def passes_check(self) -> bool:
        return self._calculate_bending_stress() < self.threshold

    def _calculate_bending_stress(self) -> float:
        lewis_y = self._get_lewis_form_factor(self.teeth_count)
        return self.effective_force / (self.face_width_mm * MODULE_MM * lewis_y * LEWIS_CORRECTION_FACTOR)

    def _get_lewis_form_factor(self, teeth: int) -> float:
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

    def _calculate_shear_stress(self) -> float:
        return self.effective_force / (self.face_width_mm * MODULE_MM)

    def get_component_von_mises(self) -> float:
        sigma_b = self._calculate_bending_stress()
        tau = self._calculate_shear_stress()
        return math.sqrt(math.pow(sigma_b, 2) + 3 * math.pow(tau, 2))


class Ring(Gear):
    def __init__(self, teeth_count: int, face_width_mm: float, tangential_force: float, radial_force: float,
                 thickness: float, threshold: float) -> None:
        super().__init__(teeth_count, face_width_mm, tangential_force, "Ring", threshold)
        self.thickness = thickness
        self.radial_force = radial_force

    def passes_check(self) -> bool:
        return self._calculate_bending_stress() < self.threshold and self._calculate_ovalization() < self.threshold

    def get_fem_loads(self) -> Dict[str, float]:
        return {"F_t (Tangential) N": self.effective_force, "F_r (Radial) N": self.radial_force}

    def _calculate_bending_stress(self) -> float:
        y_ext = self._get_lewis_form_factor(self.teeth_count)
        y_internal = 1.3 * y_ext
        return self.effective_force / (self.face_width_mm * MODULE_MM * y_internal * LEWIS_CORRECTION_FACTOR)

    def _calculate_ovalization(self) -> float:
        return (self.radial_force * self.pitch_radius_mm) / (self.thickness * self.face_width_mm)

    def get_governing_stress(self) -> float:
        sigma_b = self._calculate_bending_stress()
        sigma_o = self._calculate_ovalization()
        return math.sqrt(math.pow(sigma_b + sigma_o, 2))


class PinBase(Component):
    YOUNG_MODULUS_PLA_N_MM = 2500
    POISSONS_RATIO_PLA = 0.35
    SHEAR_MODULUS_PLA = YOUNG_MODULUS_PLA_N_MM / (2 * (1 + POISSONS_RATIO_PLA))

    def __init__(self, force_N: float, diameter_mm: float, length_mm: float, threshold: float, name: str) -> None:
        super().__init__(name, threshold)
        self.F = force_N
        self.D = diameter_mm
        self.R = diameter_mm / 2
        self.L = length_mm

    def _area(self, r: float) -> float:
        return math.pi * math.pow(r, 2)

    def _inertia(self, r: float) -> float:
        return math.pi * math.pow(r, 4) / 4

    def _bending_stresses(self) -> float:
        M = self.F * self.L / 2
        I_outer = self._inertia(self.R)
        return self._sigma(M, I_outer)

    def get_analytical_vm(self) -> float:
        return self._von_mises()

    def get_component_von_mises(self) -> float:
        return self._von_mises()

    def passes_check(self) -> bool:
        return self._von_mises() <= self.threshold

    def get_fem_loads(self) -> Dict[str, float]:
        denom = math.sqrt(1 + math.pow(math.tan(PRESSURE_ANGLE_RADIANS), 2))
        f_tangential_applied = self.F / denom
        f_radial_applied = f_tangential_applied * math.tan(PRESSURE_ANGLE_RADIANS)
        return {
            "F_t (Tangential) N": round(f_tangential_applied, 2),
            "F_r (Radial) N": round(f_radial_applied, 2),
            "Deflection mm": round(self._deflection(), 4)
        }

    @abstractmethod
    def _sigma(self, M: float, I: float) -> float:
        pass

    @abstractmethod
    def _tau(self) -> float:
        pass

    @abstractmethod
    def _von_mises(self) -> float:
        pass

    @abstractmethod
    def _deflection(self) -> float:
        pass


class Pin(PinBase):
    def __init__(self, force_N: float, diameter_mm: float, length_mm: float, fillet_radius_mm: float,
                 threshold: float) -> None:
        super().__init__(force_N, diameter_mm, length_mm, threshold, "Pin")
        self.fillet_radius = fillet_radius_mm

    def _kt(self) -> float:
        r_d = self.fillet_radius / self.D
        if r_d < 0.01: return 3.0
        if r_d < 0.05: return 2.2
        if r_d < 0.1:  return 1.7
        return 1.5

    def _von_mises(self) -> float:
        sigma = self._bending_stresses() * self._kt()
        tau = self._tau()
        return math.sqrt(math.pow(sigma, 2) + 3 * math.pow(tau, 2))

    def _deflection(self) -> float:
        I = self._inertia(self.R)
        EI = self.YOUNG_MODULUS_PLA_N_MM * I
        return self.F * math.pow(self.L, 3) / (8 * EI)

    def _sigma(self, M: float, I: float) -> float:
        return M * self.R / I

    def _tau(self) -> float:
        area = self._area(self.R)
        return 4 * self.F / (3 * area)


class SupportedPin(PinBase):
    YOUNG_MODULUS_STEEL_N_MM = 200000
    POISSONS_RATIO_STEEL = 0.30
    SHEAR_MODULUS_STEEL = YOUNG_MODULUS_STEEL_N_MM / (2 * (1 + POISSONS_RATIO_STEEL))

    def __init__(self, force_N: float, diameter_mm: float, length_mm: float, threshold: float,
                 steel_bolt_diameter_mm: float) -> None:
        super().__init__(force_N, diameter_mm, length_mm, threshold, "SupportedPin")
        self.d_bolt = steel_bolt_diameter_mm
        self.r_bolt = steel_bolt_diameter_mm / 2

    def _deflection(self) -> float:
        I_outer = self._inertia(self.R)
        I_inner = self._inertia(self.r_bolt)
        EI = (self.YOUNG_MODULUS_PLA_N_MM * (I_outer - I_inner) + self.YOUNG_MODULUS_STEEL_N_MM * I_inner)
        return self.F * math.pow(self.L, 3) / (8 * EI)

    def _von_mises(self) -> float:
        sigma = self._bending_stresses()
        tau = self._tau()
        return math.sqrt(math.pow(sigma, 2) + 3 * math.pow(tau, 2))

    def _tau(self) -> float:
        A_total = self._area(self.R)
        A_steel = self._area(self.r_bolt)
        A_pla = A_total - A_steel
        share_steel = self.SHEAR_MODULUS_STEEL * A_steel
        share_pla = self.SHEAR_MODULUS_PLA * A_pla
        F_steel = self.F * share_steel / (share_steel + share_pla)
        return 4 * F_steel / (3 * A_steel)

    def _sigma(self, M: float, I_outer: float) -> float:
        I_inner = self._inertia(self.r_bolt)
        n = self.YOUNG_MODULUS_STEEL_N_MM / self.YOUNG_MODULUS_PLA_N_MM
        I_trans = (I_outer - I_inner) + n * I_inner
        return n * M * self.r_bolt / I_trans


class CarrierHub(Component):
    def __init__(
            self,
            torque_n_mm: float,
            load_torque_n_mm: float,
            shaft_radius_mm: float,
            bolt_count: int,
            bolt_circle_radius_mm: float,
            insert_diameter_mm: float,
            insert_embed_depth_mm: float,
            threshold: float
    ) -> None:
        super().__init__("CarrierHub", threshold)
        if bolt_count <= 0:
            raise Exception("Bolts needed.")
        self.torque = torque_n_mm
        self.load_torque_n_mm = load_torque_n_mm
        self.shaft_radius = shaft_radius_mm
        self.bolt_count = bolt_count
        self.bolt_circle_radius = bolt_circle_radius_mm
        self.insert_diameter = insert_diameter_mm
        self.insert_embed_depth = insert_embed_depth_mm

    def _shaft_torsion(self) -> float:
        return (2 * self.torque) / (math.pi * math.pow(self.shaft_radius, 3))

    def _shaft_bending(self) -> float:
        return (4 * self.load_torque_n_mm) / (math.pi * math.pow(self.shaft_radius, 3))

    def _shaft_von_mises(self) -> float:
        sigma_b = self._shaft_bending()
        tau_t = self._shaft_torsion()
        return math.sqrt(math.pow(sigma_b, 2) + 3 * math.pow(tau_t, 2))

    def _bolt_shear_force(self) -> float:
        return self.torque / (self.bolt_count * self.bolt_circle_radius)

    def _bolt_tension_from_bending(self) -> float:
        return self.load_torque_n_mm / (self.bolt_count * self.bolt_circle_radius)

    def _insert_pullout(self) -> float:
        tension = self._bolt_tension_from_bending()
        shear_area = math.pi * self.insert_diameter * self.insert_embed_depth
        if shear_area == 0:
            return 0
        return tension / shear_area

    def get_component_von_mises(self) -> float:
        return max(self._shaft_von_mises(), self._insert_pullout())

    def passes_check(self) -> bool:
        return self.get_component_von_mises() < self.threshold

    def get_fem_loads(self) -> Dict[str, float]:
        shaft_tangential_force = self.torque / self.shaft_radius

        bolt_shear = self._bolt_shear_force()
        bolt_tension = self._bolt_tension_from_bending()

        return {
            "Shaft Tangential Force (N)": shaft_tangential_force,
            "Bolt Shear Force Each (N)": bolt_shear,
            "Bolt Tension Force Each (N)": bolt_tension,
            "Total Tangential Bolt Load (N)": bolt_shear * self.bolt_count,
            "Total Axial Bolt Load (N)": bolt_tension * self.bolt_count
        }


class Stage:
    def __init__(self, index: int, components: List[Component]) -> None:
        self.index = index
        self.components = components

    def get_stage_utilization(self) -> float:
        return max([c.get_margin_data()[1] for c in self.components])

    def display(self) -> None:
        print(f"Stage {self.index} results:")
        print("Component\tPass\tVM MPa\t\tU\tMoS\t\t\t\tFem\n")
        for component in self.components:
            component.display()
        print()

    def check_passed(self) -> bool:
        return all([c.passes_check() for c in self.components])


def build_stages(load_weight_kg: float, efficiency: float) -> List[Stage]:
    stages: List[Stage] = []
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
        ring = Ring(
            RING_TEETH_COUNT,
            RING_FACE_WIDTH_MM,
            ring_total_tangential,
            ring_total_radial,
            RING_WALL_THICKNESS_MM,
            MAX_SIGMA_ALLOWED_PLA
        )

        pin_force = math.sqrt(
            math.pow(2 * tangential_force, 2) +
            math.pow(2 * radial_force, 2)
        )

        pin = Pin(
            pin_force,
            PIN_DIAMETER_MM,
            PIN_LENGTH_MM,
            PIN_FILLET_RADIUS_MM,
            MAX_SIGMA_ALLOWED_PLA
        )

        pin = Pin(
            pin_force,
            PIN_DIAMETER_MM,
            PIN_LENGTH_MM,
            PIN_FILLET_RADIUS_MM,
            MAX_SIGMA_ALLOWED_PLA
        ) if i < STAGES_COUNT else SupportedPin(
            pin_force,
            PIN_DIAMETER_MM,
            PIN_LENGTH_MM,
            MAX_SIGMA_ALLOWED_STEEL,
            M3_BOLT_DIAMETER_MM
        )

        components = [sun, planet, ring, pin]

        if i == STAGES_COUNT:
            carrier_hub = CarrierHub(
                torque_n_mm=current_input_torque,
                load_torque_n_mm=load_torque,
                shaft_radius_mm=CARRIER_HUB_RADIUS_MM,
                bolt_count=CARRIER_HUB_BOLT_COUNT,
                bolt_circle_radius_mm=CARRIER_HUB_BOLT_CIRCLE_RADIUS_MM,
                insert_diameter_mm=HEAT_INSERT_DIAMETER_MM,
                insert_embed_depth_mm=HEAT_INSERT_EMBED_DEPTH_MM,
                threshold=MAX_SIGMA_ALLOWED_PLA
            )
            components.append(carrier_hub)

        stage = Stage(i, components)
        stages.append(stage)

        stage_output_torque = current_input_torque * GEAR_RATIO * efficiency
        current_input_torque = stage_output_torque

    return stages


def evaluate_system_utilization(load_weight_kg: float, efficiency: float) -> float:
    stages = build_stages(load_weight_kg, efficiency)
    return max(stage.get_stage_utilization() for stage in stages)


def display_stage_results(load_weight_kg: float, efficiency: float) -> None:
    stages = build_stages(load_weight_kg, efficiency)
    for stage in stages:
        stage.display()
        if not stage.check_passed():
            print(f"Stage {stage.index} failed stress check ❌.\n")
            return
        print(f"Stage {stage.index} passed stress check ✅.\n")


def find_max_safe_load(test_efficiency: float) -> Tuple[float, float]:
    dummy_load_kg = 1.0
    max_util = evaluate_system_utilization(dummy_load_kg, test_efficiency)
    stress_limited_load = dummy_load_kg / max_util

    total_ratio = math.pow(GEAR_RATIO, STAGES_COUNT)
    total_efficiency = math.pow(test_efficiency, STAGES_COUNT)
    torque_limited_load = (
            MOTOR_TORQUE_N_MM
            * total_ratio
            * total_efficiency
            / (GRAVITY_METER_SEC_SEC * LOAD_LEVER_ARM_MM)
    )

    final_load = min(stress_limited_load, torque_limited_load)
    final_torque = final_load * GRAVITY_METER_SEC_SEC * LOAD_LEVER_ARM_MM

    return final_load, final_torque


if __name__ == "__main__":
    print("-" * 50)
    print("1. SYSTEM CHECK")
    print("-" * 50)
    display_stage_results(LOAD_WEIGHT_KG, GEAR_EFFICIENCY)

    print("-" * 50)
    print("2. EFFICIENCY SWEEP: MAXIMUM SAFE LOAD CAPACITY")
    print("-" * 50)
    for eff in [0.8, 0.85, 0.90]:
        max_kg, max_torque = find_max_safe_load(eff)
        print(f"Efficiency {eff * 100:.0f}% | Max Safe Load: {max_kg:5.2f} kg ({max_torque:4.0f} N·mm)")
