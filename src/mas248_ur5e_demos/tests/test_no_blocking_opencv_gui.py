"""Regression checks against the old freeze-prone image_to_targets script."""
import inspect
import unittest
from scripts import image_to_targets

class ImageGUIRegressionTests(unittest.TestCase):
    def test_stage_viewer_cli_is_available(self):
        parser = image_to_targets.get_parser()
        opts = [a.option_strings for a in parser._actions]
        self.assertTrue(any('--open-stages' in item for item in opts))
        self.assertTrue(any('--no-popup' in item for item in opts))
        self.assertTrue(any('--rviz' in item for item in opts))

    def test_no_blocking_opencv_gui(self):
        source = inspect.getsource(image_to_targets)
        self.assertNotIn('cv2.imshow(', source)
        self.assertNotIn('cv2.waitKey(', source)
        self.assertNotIn('cv2.destroyAllWindows(', source)

if __name__=='__main__': unittest.main()
