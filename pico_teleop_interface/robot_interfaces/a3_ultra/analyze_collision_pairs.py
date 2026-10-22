"""离线扫描 A3 Ultra 双臂碰撞对，不参与实时遥操作。"""

from __future__ import annotations

import argparse
from collections import defaultdict

import numpy as np
import pinocchio as pin
from scipy.optimize import differential_evolution
from scipy.stats import qmc

from .dual_arm_ik import A3UltraDualArmIK, ARM_JOINT_NAMES


def _link_name(model: pin.Model, geometry: pin.GeometryObject) -> str:
    return model.frames[geometry.parentFrame].name


def _arm_indices_for_pair(pair: tuple[str, str]) -> np.ndarray:
    left = any(name.startswith(("left_", "L_")) for name in pair)
    right = any(name.startswith(("right_", "R_")) for name in pair)
    indices = []
    if left:
        indices.extend(range(7))
    if right:
        indices.extend(range(7, 14))
    return np.asarray(indices, dtype=int)


def analyze(samples: int, seed: int, exclusion_margin: float) -> list[dict]:
    """采样并优化每组连杆的最小距离，返回按距离排列的报告。"""

    ik = A3UltraDualArmIK()
    model, geometry_model = ik.model, ik._collision_model
    data, geometry_data = model.createData(), pin.GeometryData(geometry_model)
    geometry_pairs: dict[tuple[str, str], list[int]] = defaultdict(list)
    for index, pair in enumerate(geometry_model.collisionPairs):
        names = tuple(sorted((
            _link_name(model, geometry_model.geometryObjects[pair.first]),
            _link_name(model, geometry_model.geometryObjects[pair.second]),
        )))
        geometry_pairs[names].append(index)

    lower = model.lowerPositionLimit[ik._arm_q_indices]
    upper = model.upperPositionLimit[ik._arm_q_indices]
    exponent = int(np.ceil(np.log2(samples)))
    unit_samples = qmc.Sobol(14, scramble=True, seed=seed).random_base2(exponent)
    arm_samples = qmc.scale(unit_samples, lower, upper)[:samples]
    arm_samples = np.vstack((np.asarray(ik._q[ik._arm_q_indices]), arm_samples))
    minimum = {pair: float("inf") for pair in geometry_pairs}
    best_q = {pair: arm_samples[0].copy() for pair in geometry_pairs}

    for arm_q in arm_samples:
        configuration = ik._q.copy()
        configuration[ik._arm_q_indices] = arm_q
        pin.computeDistances(
            model, data, geometry_model, geometry_data, configuration)
        for pair, indices in geometry_pairs.items():
            distance = min(
                geometry_data.distanceResults[index].min_distance
                for index in indices)
            if distance < minimum[pair]:
                minimum[pair] = float(distance)
                best_q[pair] = arm_q.copy()

    # 只优化采样中离软碰撞区很远的组合，避免对明显可接触组合浪费时间。
    candidates = [pair for pair, distance in minimum.items()
                  if distance > exclusion_margin]
    for candidate_index, pair in enumerate(candidates):
        active = _arm_indices_for_pair(pair)
        if not len(active):
            continue
        collision_indices = geometry_pairs[pair]
        reference = best_q[pair].copy()

        def objective(values: np.ndarray) -> float:
            arm_q = reference.copy()
            arm_q[active] = values
            configuration = ik._q.copy()
            configuration[ik._arm_q_indices] = arm_q
            pin.updateGeometryPlacements(
                model, data, geometry_model, geometry_data, configuration)
            return min(
                pin.computeDistance(
                    geometry_model, geometry_data, index).min_distance
                for index in collision_indices)

        result = differential_evolution(
            objective,
            list(zip(lower[active], upper[active])),
            maxiter=30,
            popsize=6,
            polish=False,
            seed=seed + candidate_index,
            updating="immediate",
            workers=1,
            x0=reference[active],
        )
        minimum[pair] = min(minimum[pair], float(result.fun))

    return [
        {
            "link1": pair[0],
            "link2": pair[1],
            "minimum_distance_m": minimum[pair],
            "geometry_pairs": len(geometry_pairs[pair]),
            "classification": (
                "contact_observed" if minimum[pair] <= 0
                else "near" if minimum[pair] <= ik.collision_activation_distance
                else "keep" if minimum[pair] <= exclusion_margin
                else "srdf_candidate"
            ),
        }
        for pair in sorted(geometry_pairs, key=minimum.get)
    ]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--samples", type=int, default=8192)
    parser.add_argument("--seed", type=int, default=20261018)
    parser.add_argument("--exclusion-margin", type=float, default=0.25)
    args = parser.parse_args()
    if args.samples <= 0 or args.exclusion_margin <= 0.1:
        parser.error("samples must be positive and exclusion-margin must exceed 0.1 m")

    report = analyze(args.samples, args.seed, args.exclusion_margin)
    counts = {name: sum(row["classification"] == name for row in report)
              for name in ("contact_observed", "near", "keep", "srdf_candidate")}
    print(
        f"samples={args.samples + 1} link_pairs={len(report)} "
        + " ".join(f"{name}={count}" for name, count in counts.items()))
    print("classification,min_distance_m,geometry_pairs,link1,link2")
    for row in report:
        print(
            f"{row['classification']},{row['minimum_distance_m']:.6f},"
            f"{row['geometry_pairs']},{row['link1']},{row['link2']}")


if __name__ == "__main__":
    main()
