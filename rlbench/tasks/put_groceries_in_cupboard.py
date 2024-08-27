from typing import List, Tuple
import numpy as np
from pyrep.objects.shape import Shape
from pyrep.objects.dummy import Dummy
from pyrep.objects.object import Object
from pyrep.objects.proximity_sensor import ProximitySensor
from rlbench.backend.task import Task
from rlbench.backend.conditions import DetectedCondition, NothingGrasped
from rlbench.backend.spawn_boundary import SpawnBoundary

GROCERY_NAMES = [
    'crackers',
    'chocolate jello',
    'strawberry jello',
    'soup',
    'tuna',
    'spam',
    'coffee',
    'mustard',
    'sugar',
]


class PutGroceriesInCupboard(Task):

    def init_task(self) -> None:
        self.groceries = [Shape(name.replace(' ', '_'))
                          for name in GROCERY_NAMES]
        self.grasp_points = [Dummy('%s_grasp_point' % name.replace(' ', '_'))
                             for name in GROCERY_NAMES]
        self.waypoint1 = Dummy('waypoint1')
        self.register_graspable_objects(self.groceries)
        self.boundary = SpawnBoundary([Shape('workspace')])

    def init_episode(self, index: int) -> List[str]:
        self.boundary.clear()
        [self.boundary.sample(g, min_distance=0.1) for g in self.groceries]
        if self.use_failure_variation:
            opt_indices = list(range(0, len(self.groceries)))
            opt_indices.remove(index)
            fail_idx = np.random.choice(opt_indices)
            self.waypoint1.set_pose(self.grasp_points[fail_idx].get_pose())
        else:
            self.waypoint1.set_pose(self.grasp_points[index].get_pose())
        self.register_success_conditions(
            [DetectedCondition(self.groceries[index],
                               ProximitySensor('success')),
             NothingGrasped(self.robot.gripper)])
        return ['put the %s in the cupboard' % GROCERY_NAMES[index],
                'pick up the %s and place it in the cupboard'
                % GROCERY_NAMES[index],
                'move the %s to the bottom shelf' % GROCERY_NAMES[index],
                'put away the %s in the cupboard' % GROCERY_NAMES[index]]

    def variation_count(self) -> int:
        return len(GROCERY_NAMES)

    def boundary_root(self) -> Object:
        return Shape('boundary_root')

    def base_rotation_bounds(self) -> Tuple[Tuple[float, float, float],
                                            Tuple[float, float, float]]:
        return (0.0, 0.0, -1.), (0.0, 0.0, 1.)

