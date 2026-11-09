"""Offline student pipeline regression tests. No ROS graph or controllers needed."""
import json
from pathlib import Path
import tempfile
import unittest
from scripts.image_to_targets import convert_image,resolve_image,get_parser

class StudentImageMappingTests(unittest.TestCase):
    def _generate(self,root,**kwargs):
        root=Path(root);root.mkdir(parents=True,exist_ok=True)
        convert_image(output=str(root/'targets.json'),preview=str(root/'preview.png'),
                      stages_dir=str(root/'stages'),**kwargs)
        return json.loads((root/'targets.json').read_text())

    def test_bundled_image_and_six_stages(self):
        self.assertTrue(resolve_image().is_file())
        with tempfile.TemporaryDirectory() as d:
            data=self._generate(d)
            self.assertGreater(len(data['strokes']),0)
            self.assertEqual(len(data['stage_images']),6)
            self.assertTrue((Path(d)/'stages/index.html').exists())
            self.assertEqual(data['drawing_frame'],'image_center')
            self.assertNotIn('paper',data)

    def test_width_and_aspect_ratio(self):
        with tempfile.TemporaryDirectory() as d:
            data=self._generate(d,width_m=.2)
            w,h=data['image_size_px']
            self.assertAlmostEqual(data['image']['width_m'],.2)
            self.assertAlmostEqual(data['image']['height_m'],.2*(h-1)/(w-1))
            self.assertAlmostEqual(data['metres_per_pixel'],.2/(w-1))

    def test_world_center_shift_and_yaw(self):
        with tempfile.TemporaryDirectory() as d:
            a=self._generate(Path(d)/'a',center_x=.55,center_y=0.,center_z=.46,yaw_deg=0.)
            b=self._generate(Path(d)/'b',center_x=.75,center_y=.1,center_z=.56,yaw_deg=0.)
            pa=a['strokes'][0]['poses'][0]
            pb=b['strokes'][0]['poses'][0]
            self.assertAlmostEqual(pb['x']-pa['x'],.2,places=6)
            self.assertAlmostEqual(pb['y']-pa['y'],.1,places=6)
            self.assertAlmostEqual(pb['z']-pa['z'],.1,places=6)
            rotated=self._generate(Path(d)/'c',center_x=0.,center_y=0.,yaw_deg=90.)
            local=rotated['strokes'][0]['poses_image'][0]
            world=rotated['strokes'][0]['poses'][0]
            self.assertAlmostEqual(world['x'],-local['y'],places=6)
            self.assertAlmostEqual(world['y'],local['x'],places=6)

    def test_scale_uses_entire_image_not_contour_bbox(self):
        with tempfile.TemporaryDirectory() as d:
            data=self._generate(d,width_m=.15)
            self.assertAlmostEqual(data['image']['width_m'],.15)
            self.assertTrue(all(abs(p['x'])<=.0751 for st in data['strokes'] for p in st['poses_image']))

    def test_reject_invalid_width(self):
        with tempfile.TemporaryDirectory() as d:
            with self.assertRaises(ValueError):self._generate(d,width_m=0.)

    def test_only_image_arguments(self):
        parser=get_parser()
        opts={s for a in parser._actions for s in a.option_strings}
        for s in ('--width-m','--center-x','--center-y','--center-z'):
            self.assertIn(s,opts)
        for s in ('--paper-origin','--margin-mm','--paper-z','--paper-center-x'):
            self.assertNotIn(s,opts)

if __name__=='__main__':unittest.main()
