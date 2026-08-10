from dataclasses import dataclass


VIEWPOINT_DISTANCE = 300.0
NUM_CANDIDATES_PER_FACE = 6
ZENITH_ANGLE_DEG = 10.0

# CarDoor0808 scanner-coverage profile.  The referenced scanner note gives a
# 130 x 120 mm theoretical footprint and half-grid edge margins of 15 / 8.5 mm.
# Because the text does not identify which margin belongs to which footprint
# axis, 100 x 90 mm is the conservative cell limit valid for either mapping.
SCANNER_SAFE_PATCH_X_MM = 100.0
SCANNER_SAFE_PATCH_Y_MM = 90.0
DEFAULT_SEGMENT_U = 9
DEFAULT_SEGMENT_V = 11


@dataclass
class ABCConfig:
    food_number: int = 60
    limit: int = 150
    maximum_evaluation: int = 35000
    show_progress: bool = True
    random_seed: bool = False
    seed: int = 2025
    closed_tour: bool = False
    angle_threshold_deg: float = 30.0
    distance_weight_scale: float = 150.0
    corner_weight: float = 0.0
    visualize: bool = True
    vis_save_dir: str = "viz_frames"
    vis_interval_evals: int = 3000
    plot_history: bool = True

    @property
    def weights(self):
        return (1 / self.distance_weight_scale, self.corner_weight)


DEFAULT_ABC_CONFIG = ABCConfig()


@dataclass
class MSCGAConfig:
    pop_size: int = 200
    generations: int = 1500
    crossover_rate: float = 0.9
    mutation_rate: float = 0.08
    show_progress: bool = True

    # 2-opt local search probability per phase
    two_opt_early: float = 0.02
    two_opt_mid: float = 0.10
    two_opt_late: float = 0.08


DEFAULT_MSCGA_CONFIG = MSCGAConfig()
