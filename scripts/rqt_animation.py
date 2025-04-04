#!usr/bin/env python

import sys

from rqt_mypkg.editor import AnimationEditor
from rqt_gui.main import Main

plugin = 'rqt_animation'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))
