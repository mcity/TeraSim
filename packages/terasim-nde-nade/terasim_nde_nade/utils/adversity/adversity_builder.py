from hydra.utils import instantiate
from typing import List, cast
from omegaconf import DictConfig

from .abstract_adversity import AbstractAdversity, AbstractStaticAdversity


def _build_adversity(adversity_cfg: DictConfig) -> AbstractAdversity:
    """Build an adversity object from the configuration.

    Args:
        adversity_cfg (DictConfig): Configuration of the adversity.

    Returns:
        AbstractAdversity: Adversity object.
    """
    config = adversity_cfg.copy()
    adversity = cast(AbstractAdversity, instantiate(config))
    return adversity


def build_adversities(adversity_cfg: DictConfig) -> List[AbstractAdversity]:
    """Build adversities from the configuration.

    Args:
        adversity_cfg (DictConfig): Configuration of the adversities.
        
    Returns:
        List[AbstractAdversity]: List of adversities.
    """
    return [_build_adversity(adversity) for adversity in adversity_cfg.values()]


def _build_static_adversity(adversity_cfg: DictConfig) -> AbstractStaticAdversity:
    """Build an adversity object from the configuration.

    Args:
        adversity_cfg (DictConfig): Configuration of the adversity.

    Returns:
        AbstractAdversity: Adversity object.
    """
    config = adversity_cfg.copy()
    adversity = cast(AbstractStaticAdversity, instantiate(config))
    return adversity


def build_static_adversities(adversity_cfg: DictConfig) -> List[AbstractStaticAdversity]:
    """Build adversities from the configuration. Note that only effective static adversities are returned.

    Args:
        adversity_cfg (DictConfig): Configuration of the adversities.
        
    Returns:
        List[AbstractAdversity]: List of adversities.
    """
    static_adversity_list = []
    for cfg in adversity_cfg.values():
        static_adversity = _build_static_adversity(cfg)
        if static_adversity.is_effective():
            static_adversity_list.append(static_adversity)
    return static_adversity_list
