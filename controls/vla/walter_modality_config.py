"""GR00T modality config for the Walter cell (ABB IRB 120 + parallel gripper).

Pass to launch_finetune.py with --modality-config-path. Keys must match meta/modality.json written
by walter_control's episode_recorder (state/action: single_arm[0:6], gripper[6:7]; cameras: scene, wrist).
Follows examples/SO100/so100_config.py in Isaac-GR00T (N1.7, main branch).
"""

from gr00t.configs.data.embodiment_configs import register_modality_config
from gr00t.data.embodiment_tags import EmbodimentTag
from gr00t.data.types import ActionConfig, ActionFormat, ActionRepresentation, ActionType, ModalityConfig

walter_config = {
    "video": ModalityConfig(delta_indices=[0], modality_keys=["scene", "wrist"]),
    "state": ModalityConfig(delta_indices=[0], modality_keys=["single_arm", "gripper"]),
    "action": ModalityConfig(
        # 16 steps at 15 Hz is about 1 s of motion per prediction; the bridge executes 8 then re-plans.
        delta_indices=list(range(0, 16)),
        modality_keys=["single_arm", "gripper"],
        action_configs=[
            # Joint targets learned as deltas from the current state (generalises better).
            ActionConfig(rep=ActionRepresentation.RELATIVE, type=ActionType.NON_EEF, format=ActionFormat.DEFAULT),
            # Gripper as an absolute opening.
            ActionConfig(rep=ActionRepresentation.ABSOLUTE, type=ActionType.NON_EEF, format=ActionFormat.DEFAULT),
        ],
    ),
    "language": ModalityConfig(delta_indices=[0], modality_keys=["annotation.human.task_description"]),
}

register_modality_config(walter_config, embodiment_tag=EmbodimentTag.NEW_EMBODIMENT)
