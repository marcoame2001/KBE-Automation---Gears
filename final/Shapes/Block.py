import math
import NXOpen
import NXOpen.Annotations
import NXOpen.Features
import NXOpen.GeometricUtilities
import NXOpen.Preferences

class Block:

	def __init__(self, x, y,z, length, width, height, xaxis, yaxis):
		self.x = x
		self.y = y
		self.z = z
		self.length = length  # instance variable unique to each instance
		self.width = width
		self.height = height
		self.xaxis = xaxis
		self.yaxis = yaxis
		
		self.initForNX()

	def initForNX(self):
		theSession = NXOpen.Session.GetSession()
		workPart = theSession.Parts.Work

		#   The block
		blockfeaturebuilder1 = workPart.Features.CreateBlockFeatureBuilder(NXOpen.Features.Block.Null)
		blockfeaturebuilder1.Type = NXOpen.Features.BlockFeatureBuilder.Types.OriginAndEdgeLengths

		origBlock = NXOpen.Point3d(float(self.x), float(self.y), float(self.z))
		blockfeaturebuilder1.SetOriginAndLengths(origBlock, str(self.length), str(self.width), str(self.height))
		blockfeaturebuilder1.BooleanOption.Type = NXOpen.GeometricUtilities.BooleanOperation.BooleanType.Create
		blockfeaturebuilder1.SetOrientation(self.xaxis, self.yaxis)

		self.body = blockfeaturebuilder1.Commit().GetBodies()[0]
		blockfeaturebuilder1.Destroy()
