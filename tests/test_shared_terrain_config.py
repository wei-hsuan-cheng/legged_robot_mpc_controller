"""Shared terrain paths, matching geometry and robot-specific planner settings."""

import ast
from pathlib import Path
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[1]
TERRAINS = ROOT / 'config/terrains'


class SharedTerrainConfigTest(unittest.TestCase):
    def test_both_launches_select_their_shared_variants(self):
        launch = ast.parse((ROOT / 'launch/robot.launch.py').read_text())
        for robot in ('g1', 'h2'):
            defaults = {}
            for node in ast.walk(launch):
                if not isinstance(node, ast.Assign) or not isinstance(node.value, ast.Call):
                    continue
                for target in node.targets:
                    if isinstance(target, ast.Name) and target.id in (
                            'stair_climbing_file_default', 'terrain_walking_file_default'):
                        defaults[target.id] = [part.value if isinstance(part, ast.Constant) else f'{robot}.yaml'
                                               for part in node.value.args[0].elts
                                               if isinstance(part, (ast.Constant, ast.List))]
            self.assertEqual(defaults, {
                'stair_climbing_file_default': ['config', 'terrains', 'stair_climbing', f'{robot}.yaml'],
                'terrain_walking_file_default': ['config', 'terrains', 'terrain_walking', f'{robot}.yaml'],
            })
            for parts in defaults.values():
                self.assertTrue(ROOT.joinpath(*parts).is_file())
            self.assertFalse(list((ROOT / 'config' / robot / 'terrain').rglob('*.yaml')))

    def test_all_variants_match_the_shared_scene_staircase(self):
        scene = yaml.safe_load((TERRAINS / 'stairs_params.yaml').read_text())['stairs']
        expected = dict(base_pos=scene['base']['pos'], yaw=scene['base']['yaw'],
                        start_offset=scene['base']['start_offset'], heights=scene['heights'],
                        depths=scene['depths'], width=scene['widths'][0])
        self.assertEqual(len(scene['heights']), scene['num_steps'])
        self.assertTrue(all(w == expected['width'] for w in scene['widths']))
        for mode, key in (('stair_climbing', 'stair_climbing'), ('terrain_walking', 'terrain_walk')):
            for robot in ('g1', 'h2'):
                config = yaml.safe_load((TERRAINS / mode / f'{robot}.yaml').read_text())
                self.assertEqual(config[key]['stairs'], expected, (mode, robot))

    def test_robot_dimensions_are_not_replaced_by_one_common_value(self):
        for robot, lateral, stair_height, margin, max_height in (
                ('g1', .115, .70, .10, .80), ('h2', .14, .90, .12, 1.03)):
            stair = yaml.safe_load((TERRAINS / 'stair_climbing' / f'{robot}.yaml').read_text())['stair_climbing']
            walk = yaml.safe_load((TERRAINS / 'terrain_walking' / f'{robot}.yaml').read_text())['terrain_walk']
            self.assertEqual(stair['footholds']['lateral_offset'], lateral)
            self.assertEqual(stair['base']['height_above_support'], stair_height)
            self.assertEqual(walk['hip_lateral_offset'], lateral)
            self.assertEqual(walk['foot_margin_x'], margin)
            self.assertEqual(walk['max_base_height_above_support'], max_height)


if __name__ == '__main__':
    unittest.main()
