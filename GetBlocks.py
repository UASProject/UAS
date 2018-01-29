from pixy import *
from ctypes import *

# Initialize Pixy Interpreter thread #
pixy_init() #comment out if function is in main file

def GetBlocks():


	class Blocks (Structure):
		fields_ = [ ("type", c_uint),
		("signature", c_uint),
		("x", c_uint),
		("y", c_uint),
		("width", c_uint),
		("height", c_uint),
		("angle", c_uint) ]
	
	blocks = BlockArray(100)
	frame  = 0
	
	# Wait for blocks #
	while 1:
	
		count = pixy_get_blocks(100, blocks)
		
		if count > 0:
		# Blocks found #
			print 'frame %3d:' % (frame)
			frame = frame + 1
			for index in range (0, count):
				x=block[index].x
				y=block[index].y
				print '[BLOCK_TYPE=%d SIG=%d X=%3d Y=%3d WIDTH=%3d HEIGHT=%3d]' % (blocks[index].type, blocks[index].signature, blocks[index].x, blocks[index].y, blocks[index].width, blocks[index].height)
				print(x)
				print(y)
				return x,y;

x,y=GetBlocks()
