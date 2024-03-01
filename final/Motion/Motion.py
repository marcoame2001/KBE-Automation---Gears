#Motion class - second attempt based on "clean" journal
#BE CAREFUL WITH THE PATHS
import math
import NXOpen
import NXOpen.Assemblies
import NXOpen.CAE
import NXOpen.MenuBar
import NXOpen.Motion
import random

class Motion:

    '''
    Suggestions:
    velocity, accel., dir.
    sim.time
    bodies
    joints
    lock? (constrains)
    service params - file paths
    '''
    def __init__(self, bodies, driver, simTime, velocity, accel, dir, folder, fileName, number_list, direction):
        self.bodies = bodies #Bodies to be moved (joint derived accordingly)
        self.driver = driver #Index of a driver body in bodies
        self.simTime = simTime # Simulation time - seconds
        self.velocity = velocity #Velocity of a driver body - degrees/sec
        self.accel = accel # acceleration of the driver - degrees/pow(sec, 2)
        self.dir = dir # direction of rotation
        self.folder = folder #folder to store sim results
        self.fileName = fileName #model file name.
        self.number_list = number_list
        self.direction = direction
        self.nxObjectBodies = [] #to store NX Bodies
        self.nxObjectJoints = [] #to store NX Joints
        self.driver = None 
        
        #Lets also initialize some key things here
        theSession  = NXOpen.Session.GetSession()
        self.theSession  = theSession
        self.workPart = theSession.Parts.Work
        self.displayPart = theSession.Parts.Display
        
        self.initForNX()
        
    def defineMotionBodies(self, bodies):
        #Basic references
        theSession  = NXOpen.Session.GetSession()
        workPart = theSession.Parts.Work
        
        i = 0
        for body in bodies:
            
            linkBuilder1 = workPart.MotionManager.Links.CreateLinkBuilder(NXOpen.Motion.Link.Null)
            linkBuilder1.MassProperty.MassType = NXOpen.Motion.LinkMassProperty.MassPropertyType.UserDefined
            
            linkBuilder1.MassProperty.MassExpression.SetFormula("1") # Mass in kg
            linkBuilder1.MassProperty.IxxExpression.SetFormula("1") # Inertia tensor matrix elements
            linkBuilder1.MassProperty.IyyExpression.SetFormula("1")
            linkBuilder1.MassProperty.IzzExpression.SetFormula("1")
            linkBuilder1.MassProperty.IxyExpression.SetFormula("0")
            linkBuilder1.MassProperty.IxzExpression.SetFormula("0")
            linkBuilder1.MassProperty.IyzExpression.SetFormula("0")
            
            try:
                # This expression cannot be modified because it is locked.
                linkBuilder1.MassProperty.AutoMassExpression.SetFormula("0")
            except NXOpen.NXException as ex:
                ex.AssertErrorCode(1050049)
                
            try:
                # This expression cannot be modified because it is locked.
                linkBuilder1.MassProperty.AutoIxxExpression.SetFormula("0")
            except NXOpen.NXException as ex:
                ex.AssertErrorCode(1050049)
                
            try:
                # This expression cannot be modified because it is locked.
                linkBuilder1.MassProperty.AutoIyyExpression.SetFormula("0")
            except NXOpen.NXException as ex:
                ex.AssertErrorCode(1050049)
                
            try:
                # This expression cannot be modified because it is locked.
                linkBuilder1.MassProperty.AutoIzzExpression.SetFormula("0")
            except NXOpen.NXException as ex:
                ex.AssertErrorCode(1050049)
                
            try:
                # This expression cannot be modified because it is locked.
                linkBuilder1.MassProperty.AutoIxyExpression.SetFormula("0")
            except NXOpen.NXException as ex:
                ex.AssertErrorCode(1050049)
                
            try:
                # This expression cannot be modified because it is locked.
                linkBuilder1.MassProperty.AutoIxzExpression.SetFormula("0")
            except NXOpen.NXException as ex:
                ex.AssertErrorCode(1050049)
                
            try:
                # This expression cannot be modified because it is locked.
                linkBuilder1.MassProperty.AutoIyzExpression.SetFormula("0")
            except NXOpen.NXException as ex:
                ex.AssertErrorCode(1050049)
                
            linkBuilder1.InitialVelocity.TranslateExpression.SetFormula("0")
            linkBuilder1.InitialVelocity.RotateExpression.SetFormula("0")
            linkBuilder1.InitialVelocity.WxExpression.SetFormula("0")
            linkBuilder1.InitialVelocity.WyExpression.SetFormula("0")
            linkBuilder1.InitialVelocity.WzExpression.SetFormula("0")
            
            unit1 = workPart.UnitCollection.FindObject("MilliMeter")
            expression1 = workPart.Expressions.CreateSystemExpressionWithUnits("0", unit1)
            expression2 = workPart.Expressions.CreateSystemExpressionWithUnits("0", unit1)
            
            globalSelectionBuilder2 = theSession.MotionSession.MotionMethods.GetGlobalSelectionBuilder(workPart)
            selectTaggedObjectList1 = globalSelectionBuilder2.Selection
            
            #So, here "cylinders" are searched for.
            component1 = workPart.ComponentAssembly.RootComponent.FindObject("COMPONENT " + self.fileName + " 1")
            body1 = component1.FindObject("PROTO#.Bodies|CYLINDER(" + str(self.number_list[i]) + ")")
            i=i+1
            added1 = linkBuilder1.Geometries.Add(body1)
            
            globalSelectionBuilder3 = theSession.MotionSession.MotionMethods.GetGlobalSelectionBuilder(workPart)
            selectTaggedObjectList2 = globalSelectionBuilder3.Selection
            
            linkBuilder1.InitialVelocity.TranslateVector = NXOpen.Direction.Null
            linkBuilder1.InitialVelocity.RotateVector = NXOpen.Direction.Null
            
            nxObjectMotionBody = linkBuilder1.Commit()
            self.nxObjectBodies.append(nxObjectMotionBody) #NX object for the motion body is created and added to the list
            
            
            linkBuilder1.Destroy()
            workPart.MeasureManager.SetPartTransientModification()
            workPart.Expressions.Delete(expression1)
            workPart.MeasureManager.ClearPartTransientModification()
            workPart.MeasureManager.SetPartTransientModification()
            workPart.Expressions.Delete(expression2)
            workPart.MeasureManager.ClearPartTransientModification()
            theSession.CleanUpFacetedFacesAndEdges()

    def defineJoints(self, bodies):
        #Basic references
        theSession  = NXOpen.Session.GetSession()
        workPart = theSession.Parts.Work
        component1 = self.component1
        unit1 = self.unit1
        
        i = 0 # amount of bodies = amount of joints
        for body in bodies:
            
            jointBuilder1 = workPart.MotionManager.Joints.CreateJointBuilder(NXOpen.Motion.Joint.Null)
            jointBuilder1.JointDefine.UpperLimitExpression.SetFormula("0")
            jointBuilder1.JointDefine.LowerLimitExpression.SetFormula("0")
            jointBuilder1.JointDefine.UpperLimitAngleExpression.SetFormula("0")
            jointBuilder1.JointDefine.LowerLimitAngleExpression.SetFormula("0")
            jointBuilder1.JointDefine.ScrewRatioExpression.SetFormula("1")
            jointBuilder1.JointFriction.AdamsFriction.MuStaticExpression.SetFormula("0")
            jointBuilder1.JointFriction.AdamsFriction.MuDynamicExpression.SetFormula("0.1")
            jointBuilder1.JointFriction.AdamsFriction.StictionTransitionVelocityExpression.SetFormula("0.1")
            jointBuilder1.JointFriction.AdamsFriction.MaxStictionDeformationExpression.SetFormula("0.01")
            jointBuilder1.JointFriction.AdamsFriction.BallRadiusExpression.SetFormula("1")
            jointBuilder1.JointFriction.AdamsFriction.PinRadiusExpression.SetFormula("1")
            jointBuilder1.JointFriction.AdamsFriction.BendingArmExpression.SetFormula("1")
            jointBuilder1.JointFriction.AdamsFriction.FrictionArmExpression.SetFormula("1")
            jointBuilder1.JointFriction.AdamsFriction.ReactionArmExpression.SetFormula("1")
            jointBuilder1.JointFriction.AdamsFriction.InitialOverlapExpression.SetFormula("1000")
            jointBuilder1.JointFriction.AdamsFriction.ForcePreloadExpression.SetFormula("0")
            jointBuilder1.JointFriction.AdamsFriction.TorquePreloadExpression.SetFormula("0")
            jointBuilder1.JointFriction.RecurDynFriction.MuStaticExpression.SetFormula("0")
            jointBuilder1.JointFriction.RecurDynFriction.MuDynamicExpression.SetFormula("0.1")
            jointBuilder1.JointFriction.RecurDynFriction.StictionTransitionVelocityExpression.SetFormula("0.1")
            jointBuilder1.JointFriction.RecurDynFriction.MaxStictionDeformationExpression.SetFormula("0.01")
            jointBuilder1.JointFriction.RecurDynFriction.BallRadiusExpression.SetFormula("1")
            jointBuilder1.JointFriction.RecurDynFriction.PinRadiusExpression.SetFormula("1")
            jointBuilder1.JointFriction.RecurDynFriction.BendingArmExpression.SetFormula("1")
            jointBuilder1.JointFriction.RecurDynFriction.FrictionArmExpression.SetFormula("1")
            jointBuilder1.JointFriction.RecurDynFriction.ReactionArmExpression.SetFormula("10")
            jointBuilder1.JointFriction.RecurDynFriction.InitialOverlapExpression.SetFormula("1000")
            jointBuilder1.JointFriction.RecurDynFriction.ForcePreloadExpression.SetFormula("0")
            jointBuilder1.JointFriction.RecurDynFriction.TorquePreloadExpression.SetFormula("0")
            jointBuilder1.JointFriction.RecurDynFriction.MaxFrictionForceExpression.SetFormula("0")
            jointBuilder1.JointFriction.RecurDynFriction.MaxFrictionTorqueExpression.SetFormula("0")
            jointBuilder1.JointFriction.LmsFriction.MuStatic.SetFormula("0")
            jointBuilder1.JointFriction.LmsFriction.MuDynamic.SetFormula("0.1")
            jointBuilder1.JointFriction.LmsFriction.TranslationalStictionTransitionVelocity.SetFormula("0.1")
            jointBuilder1.JointFriction.LmsFriction.RotationalStictionTransitionVelocity.SetFormula("0.0572957795130824")
            jointBuilder1.JointFriction.LmsFriction.PinRadius.SetFormula("1")
            jointBuilder1.JointFriction.LmsFriction.BendingReactionArm.SetFormula("1")
            jointBuilder1.JointFriction.LmsFriction.FrictionArm.SetFormula("1")
            jointBuilder1.JointFriction.LmsFriction.ReactionArm.SetFormula("10")
            jointBuilder1.JointFriction.LmsFriction.InitialOverlap.SetFormula("1000")
            jointBuilder1.JointFriction.LmsFriction.BallRadius.SetFormula("1")
            jointBuilder1.JointMultiDrivers.MotionEulerAngle1.DisplacementExpression.SetFormula("0")
            jointBuilder1.JointMultiDrivers.MotionEulerAngle1.VelocityExpression.SetFormula("0")
            jointBuilder1.JointMultiDrivers.MotionEulerAngle1.AccelerationExpression.SetFormula("0")
            jointBuilder1.JointMultiDrivers.MotionEulerAngle1.JerkExpression.SetFormula("0")
            jointBuilder1.JointMultiDrivers.MotionEulerAngle1.AmplitudeExpression.SetFormula("0")
            jointBuilder1.JointMultiDrivers.MotionEulerAngle1.FrequencyExpression.SetFormula("0")
            jointBuilder1.JointMultiDrivers.MotionEulerAngle1.PhaseAngleExpression.SetFormula("0")
            jointBuilder1.JointMultiDrivers.MotionEulerAngle1.HarmonicDisplacementExpression.SetFormula("0")
            jointBuilder1.JointMultiDrivers.MotionEulerAngle1.InitialDisplacementExpression.SetFormula("0")
            jointBuilder1.JointMultiDrivers.MotionEulerAngle1.InitialVelocityExpression.SetFormula("0")
            #jointBuilder1.JointMultiDrivers.MotionEulerAngle1.ControlInitialVelocityExpression.SetFormula("0")
            #jointBuilder1.JointMultiDrivers.MotionEulerAngle1.ControlInitialAccelerationExpression.SetFormula("0")
            jointBuilder1.JointMultiDrivers.MotionTranslationZ.DisplacementExpression.SetFormula("0")
            jointBuilder1.JointMultiDrivers.MotionTranslationZ.VelocityExpression.SetFormula("0")
            jointBuilder1.JointMultiDrivers.MotionTranslationZ.AccelerationExpression.SetFormula("0")
            jointBuilder1.JointMultiDrivers.MotionTranslationZ.JerkExpression.SetFormula("0")
            jointBuilder1.JointMultiDrivers.MotionTranslationZ.AmplitudeExpression.SetFormula("0")
            jointBuilder1.JointMultiDrivers.MotionTranslationZ.FrequencyExpression.SetFormula("0")
            jointBuilder1.JointMultiDrivers.MotionTranslationZ.PhaseAngleExpression.SetFormula("0")
            jointBuilder1.JointMultiDrivers.MotionTranslationZ.HarmonicDisplacementExpression.SetFormula("0")
            jointBuilder1.JointMultiDrivers.MotionTranslationZ.InitialDisplacementExpression.SetFormula("0")
            jointBuilder1.JointMultiDrivers.MotionTranslationZ.InitialVelocityExpression.SetFormula("0")
            #jointBuilder1.JointMultiDrivers.MotionTranslationZ.ControlInitialVelocityExpression.SetFormula("0")
            #jointBuilder1.JointMultiDrivers.MotionTranslationZ.ControlInitialAccelerationExpression.SetFormula("0")
            jointBuilder1.JointMultiDrivers.MotionPointOnCurve.DisplacementExpression.SetFormula("0")
            jointBuilder1.JointMultiDrivers.MotionPointOnCurve.VelocityExpression.SetFormula("0")
            jointBuilder1.JointMultiDrivers.MotionPointOnCurve.AccelerationExpression.SetFormula("0")
            jointBuilder1.JointMultiDrivers.MotionPointOnCurve.JerkExpression.SetFormula("0")
            jointBuilder1.JointMultiDrivers.MotionPointOnCurve.AmplitudeExpression.SetFormula("0")
            jointBuilder1.JointMultiDrivers.MotionPointOnCurve.FrequencyExpression.SetFormula("0")
            jointBuilder1.JointMultiDrivers.MotionPointOnCurve.PhaseAngleExpression.SetFormula("0")
            jointBuilder1.JointMultiDrivers.MotionPointOnCurve.HarmonicDisplacementExpression.SetFormula("0")
            jointBuilder1.JointMultiDrivers.MotionPointOnCurve.InitialDisplacementExpression.SetFormula("0")
            jointBuilder1.JointMultiDrivers.MotionPointOnCurve.InitialVelocityExpression.SetFormula("0")
            #jointBuilder1.JointMultiDrivers.MotionPointOnCurve.ControlInitialVelocityExpression.SetFormula("0")
            #jointBuilder1.JointMultiDrivers.MotionPointOnCurve.ControlInitialAccelerationExpression.SetFormula("0")
            
            expression5 = workPart.Expressions.CreateSystemExpressionWithUnits("0", unit1)
            expression6 = workPart.Expressions.CreateSystemExpressionWithUnits("0", unit1)
            expression7 = workPart.Expressions.CreateSystemExpressionWithUnits("0", unit1)
            expression8 = workPart.Expressions.CreateSystemExpressionWithUnits("0", unit1)
            expression9 = workPart.Expressions.CreateSystemExpressionWithUnits("0", unit1)
            expression10 = workPart.Expressions.CreateSystemExpressionWithUnits("0", unit1)
            
            jointBuilder1.JointDefine.ScrewSplineFunction = NXOpen.CAE.Function.Null
            jointBuilder1.JointDefine.ScrewDisplCurveFunction = NXOpen.CAE.Function.Null
            jointBuilder1.JointMultiDrivers.MotionEulerAngle1.Function = NXOpen.NXObject.Null
            jointBuilder1.JointMultiDrivers.MotionEulerAngle1.Function = NXOpen.NXObject.Null
            jointBuilder1.JointMultiDrivers.MotionTranslationZ.Function = NXOpen.NXObject.Null
            jointBuilder1.JointMultiDrivers.MotionTranslationZ.Function = NXOpen.NXObject.Null
            jointBuilder1.JointMultiDrivers.MotionPointOnCurve.Function = NXOpen.NXObject.Null
            jointBuilder1.JointMultiDrivers.MotionPointOnCurve.Function = NXOpen.NXObject.Null
            jointBuilder1.JointMultiDrivers.MotionEulerAngle1.Function = NXOpen.NXObject.Null
            jointBuilder1.JointMultiDrivers.MotionTranslationZ.Function = NXOpen.NXObject.Null
            jointBuilder1.JointMultiDrivers.MotionPointOnCurve.Function = NXOpen.NXObject.Null
            
            globalSelectionBuilder6 = theSession.MotionSession.MotionMethods.GetGlobalSelectionBuilder(workPart)
            selectTaggedObjectList5 = globalSelectionBuilder6.Selection
            globalSelectionBuilder7 = theSession.MotionSession.MotionMethods.GetGlobalSelectionBuilder(workPart)
            selectTaggedObjectList6 = globalSelectionBuilder7.Selection
            globalSelectionBuilder8 = theSession.MotionSession.MotionMethods.GetGlobalSelectionBuilder(workPart)
            selectTaggedObjectList7 = globalSelectionBuilder8.Selection
            
            #Assigning body to the joint, i-1 as the indexing must start with 0, while component index from 1...
            link1 = self.nxObjectBodies[i] #nXObject2
            jointBuilder1.JointDefine.FirstLinkSelection.Value = link1
            
            # Getting right object and right coordinate for the joint on it! Cylinders placed along Y axis.
            face1 = component1.FindObject("PROTO#.Features|CYLINDER("+str(self.number_list[i])+")|FACE 1 {("+ str(self.bodies[i].x) +"," + str(self.bodies[i].y) + "," + str(self.bodies[i].z + self.bodies[i].height)  + ") CYLINDER("+str(self.number_list[i])+")}")
            #That how it was in the journal recorded for 2 cylinders of Diameters 200 and 50. So for the second cylider, y is 125 (= 200 / 2 + 50 / 2).
            #face1 = component1.FindObject("PROTO#.Features|CYLINDER(1)|FACE 1 {(0,0,10) CYLINDER(1)}")
            #face3 = component1.FindObject("PROTO#.Features|CYLINDER(2)|FACE 1 {(0,125,10) CYLINDER(2)}")
            i=i+1
            xform1, nXObject4 = workPart.Xforms.CreateExtractXform(face1, NXOpen.SmartObject.UpdateOption.AfterModeling, False)
            
            scalar1 = workPart.Scalars.CreateScalar(0.5, NXOpen.Scalar.DimensionalityType.NotSet, NXOpen.SmartObject.UpdateOption.AfterModeling)
            
            scalar2 = workPart.Scalars.CreateScalar(0.5, NXOpen.Scalar.DimensionalityType.NotSet, NXOpen.SmartObject.UpdateOption.AfterModeling)
            
            face2 = nXObject4
            point1 = workPart.Points.CreatePoint(face2, scalar1, scalar2, NXOpen.SmartObject.UpdateOption.AfterModeling)
            
            point2 = workPart.Points.CreatePoint(point1, xform1, NXOpen.SmartObject.UpdateOption.AfterModeling)
            
            jointBuilder1.JointDefine.FirstOrigin = point2
            
            point3 = workPart.Points.CreatePoint(point1, xform1, NXOpen.SmartObject.UpdateOption.AfterModeling)
            
            globalSelectionBuilder9 = theSession.MotionSession.MotionMethods.GetGlobalSelectionBuilder(workPart)
            
            selectTaggedObjectList8 = globalSelectionBuilder9.Selection
            
            globalSelectionBuilder10 = theSession.MotionSession.MotionMethods.GetGlobalSelectionBuilder(workPart)
            
            selectTaggedObjectList9 = globalSelectionBuilder10.Selection
            
            globalSelectionBuilder11 = theSession.MotionSession.MotionMethods.GetGlobalSelectionBuilder(workPart)
            
            selectTaggedObjectList10 = globalSelectionBuilder11.Selection
            
            origin1 = NXOpen.Point3d(0.0, 0.0, 0.0)
            vector1 = NXOpen.Vector3d(0.0, 0.0, self.direction)
            direction1 = workPart.Directions.CreateDirection(origin1, vector1, NXOpen.SmartObject.UpdateOption.AfterModeling)
            
            jointBuilder1.JointDefine.FirstVector = direction1
            
            nxObjectJoint = jointBuilder1.Commit() # Joint created / nXObject5
            self.nxObjectJoints.append(nxObjectJoint)
            
            jointBuilder1.Destroy()
            workPart.Points.DeletePoint(point3)
            workPart.MeasureManager.SetPartTransientModification()
            workPart.Expressions.Delete(expression5)
            workPart.MeasureManager.ClearPartTransientModification()
            workPart.MeasureManager.SetPartTransientModification()
            workPart.Expressions.Delete(expression6)
            workPart.MeasureManager.ClearPartTransientModification()
            workPart.MeasureManager.SetPartTransientModification()
            workPart.Expressions.Delete(expression7)
            workPart.MeasureManager.ClearPartTransientModification()
            workPart.MeasureManager.SetPartTransientModification()
            workPart.Expressions.Delete(expression8)
            workPart.MeasureManager.ClearPartTransientModification()
            workPart.MeasureManager.SetPartTransientModification()
            workPart.Expressions.Delete(expression9)
            workPart.MeasureManager.ClearPartTransientModification()
            workPart.MeasureManager.SetPartTransientModification()
            workPart.Expressions.Delete(expression10)
            workPart.MeasureManager.ClearPartTransientModification()
            theSession.CleanUpFacetedFacesAndEdges()
    
    # indexOfDriver defines the number of the driver joint
    def defineDriver(self, indexOfDriver, velocity, accel, dir):
        #Basic references
        theSession  = NXOpen.Session.GetSession()
        workPart = theSession.Parts.Work
        unit1 = self.unit1
        
        jointDriverBuilder1 = workPart.MotionManager.JointDrivers.CreateJointDriverBuilder(NXOpen.Motion.JointDriver.Null)
        linkDriverBuilder1 = workPart.MotionManager.LinkDrivers.CreateLinkDriverBuilder(NXOpen.Motion.LinkDriver.Null)
        jointDriverBuilder1.DriverMultiOperations.MotionEulerAngle1.TypeOption = NXOpen.Motion.DriverOperation.Type.Polynomial

        jointDriverBuilder1.DriverMultiOperations.MotionEulerAngle1.DisplacementExpression.SetFormula("0")
        jointDriverBuilder1.DriverMultiOperations.MotionEulerAngle1.VelocityExpression.SetFormula(str(velocity)) #set the velocity
        jointDriverBuilder1.DriverMultiOperations.MotionEulerAngle1.AccelerationExpression.SetFormula(str(accel))
        jointDriverBuilder1.DriverMultiOperations.MotionEulerAngle1.JerkExpression.SetFormula("0")
        
        jointDriverBuilder1.DriverMultiOperations.MotionEulerAngle1.AmplitudeExpression.SetFormula("0")
        
        jointDriverBuilder1.DriverMultiOperations.MotionEulerAngle1.FrequencyExpression.SetFormula("0")
        
        jointDriverBuilder1.DriverMultiOperations.MotionEulerAngle1.PhaseAngleExpression.SetFormula("0")
        
        jointDriverBuilder1.DriverMultiOperations.MotionEulerAngle1.HarmonicDisplacementExpression.SetFormula("0")
        
        jointDriverBuilder1.DriverMultiOperations.MotionEulerAngle1.InitialDisplacementExpression.SetFormula("0")
        
        jointDriverBuilder1.DriverMultiOperations.MotionEulerAngle1.InitialVelocityExpression.SetFormula("0")
        
        #jointDriverBuilder1.DriverMultiOperations.MotionEulerAngle1.ControlInitialVelocityExpression.SetFormula("10")
        
        #jointDriverBuilder1.DriverMultiOperations.MotionEulerAngle1.ControlInitialAccelerationExpression.SetFormula("0")
        
        jointDriverBuilder1.DriverMultiOperations.MotionTranslationZ.DisplacementExpression.SetFormula("0")
        
        jointDriverBuilder1.DriverMultiOperations.MotionTranslationZ.VelocityExpression.SetFormula("0")
        
        jointDriverBuilder1.DriverMultiOperations.MotionTranslationZ.AccelerationExpression.SetFormula("0")
        
        jointDriverBuilder1.DriverMultiOperations.MotionTranslationZ.JerkExpression.SetFormula("0")
        
        jointDriverBuilder1.DriverMultiOperations.MotionTranslationZ.AmplitudeExpression.SetFormula("0")
        
        jointDriverBuilder1.DriverMultiOperations.MotionTranslationZ.FrequencyExpression.SetFormula("0")
        
        jointDriverBuilder1.DriverMultiOperations.MotionTranslationZ.PhaseAngleExpression.SetFormula("0")
        
        jointDriverBuilder1.DriverMultiOperations.MotionTranslationZ.HarmonicDisplacementExpression.SetFormula("0")
        
        jointDriverBuilder1.DriverMultiOperations.MotionTranslationZ.InitialDisplacementExpression.SetFormula("0")
        
        jointDriverBuilder1.DriverMultiOperations.MotionTranslationZ.InitialVelocityExpression.SetFormula("0")
        
        #jointDriverBuilder1.DriverMultiOperations.MotionTranslationZ.ControlInitialVelocityExpression.SetFormula("0")
        
        #jointDriverBuilder1.DriverMultiOperations.MotionTranslationZ.ControlInitialAccelerationExpression.SetFormula("0")
        
        jointDriverBuilder1.DriverMultiOperations.MotionPointOnCurve.TypeOption = NXOpen.Motion.DriverOperation.Type.Undefined
        
        jointDriverBuilder1.DriverMultiOperations.MotionPointOnCurve.DisplacementExpression.SetFormula("0")
        
        jointDriverBuilder1.DriverMultiOperations.MotionPointOnCurve.VelocityExpression.SetFormula("0")
        
        jointDriverBuilder1.DriverMultiOperations.MotionPointOnCurve.AccelerationExpression.SetFormula("0")
        
        jointDriverBuilder1.DriverMultiOperations.MotionPointOnCurve.JerkExpression.SetFormula("0")
        
        jointDriverBuilder1.DriverMultiOperations.MotionPointOnCurve.AmplitudeExpression.SetFormula("0")
        
        jointDriverBuilder1.DriverMultiOperations.MotionPointOnCurve.FrequencyExpression.SetFormula("0")
        
        jointDriverBuilder1.DriverMultiOperations.MotionPointOnCurve.PhaseAngleExpression.SetFormula("0")
        
        jointDriverBuilder1.DriverMultiOperations.MotionPointOnCurve.HarmonicDisplacementExpression.SetFormula("0")
        
        jointDriverBuilder1.DriverMultiOperations.MotionPointOnCurve.InitialDisplacementExpression.SetFormula("0")
        
        jointDriverBuilder1.DriverMultiOperations.MotionPointOnCurve.InitialVelocityExpression.SetFormula("0")
        
        #jointDriverBuilder1.DriverMultiOperations.MotionPointOnCurve.ControlInitialVelocityExpression.SetFormula("0")
        
        #jointDriverBuilder1.DriverMultiOperations.MotionPointOnCurve.ControlInitialAccelerationExpression.SetFormula("0")
        
        linkDriverBuilder1.BaseLinkAttachment.OrientationType = NXOpen.Motion.LinkAttachmentData.OrientationTypes.Vector
        
        linkDriverBuilder1.DriverMotions.RotationX.DisplacementExpression.SetFormula("0")
        
        linkDriverBuilder1.DriverMotions.RotationX.VelocityExpression.SetFormula("0")
        
        linkDriverBuilder1.DriverMotions.RotationX.AccelerationExpression.SetFormula("0")
        
        linkDriverBuilder1.DriverMotions.RotationX.JerkExpression.SetFormula("0")
        
        linkDriverBuilder1.DriverMotions.RotationX.AmplitudeExpression.SetFormula("0")
        
        linkDriverBuilder1.DriverMotions.RotationX.FrequencyExpression.SetFormula("0")
        
        linkDriverBuilder1.DriverMotions.RotationX.PhaseAngleExpression.SetFormula("0")
        
        linkDriverBuilder1.DriverMotions.RotationX.HarmonicDisplacementExpression.SetFormula("0")
        
        linkDriverBuilder1.DriverMotions.RotationX.InitialDisplacementExpression.SetFormula("0")
        
        linkDriverBuilder1.DriverMotions.RotationX.InitialVelocityExpression.SetFormula("0")
        
        #linkDriverBuilder1.DriverMotions.RotationX.ControlInitialVelocityExpression.SetFormula("0")
        
        #linkDriverBuilder1.DriverMotions.RotationX.ControlInitialAccelerationExpression.SetFormula("0")
        
        linkDriverBuilder1.DriverMotions.RotationY.DisplacementExpression.SetFormula("0")
        
        linkDriverBuilder1.DriverMotions.RotationY.VelocityExpression.SetFormula("0")
        
        linkDriverBuilder1.DriverMotions.RotationY.AccelerationExpression.SetFormula("0")
        
        linkDriverBuilder1.DriverMotions.RotationY.JerkExpression.SetFormula("0")
        
        linkDriverBuilder1.DriverMotions.RotationY.AmplitudeExpression.SetFormula("0")
        
        linkDriverBuilder1.DriverMotions.RotationY.FrequencyExpression.SetFormula("0")
        
        linkDriverBuilder1.DriverMotions.RotationY.PhaseAngleExpression.SetFormula("0")
        
        linkDriverBuilder1.DriverMotions.RotationY.HarmonicDisplacementExpression.SetFormula("0")
        
        linkDriverBuilder1.DriverMotions.RotationY.InitialDisplacementExpression.SetFormula("0")
        
        linkDriverBuilder1.DriverMotions.RotationY.InitialVelocityExpression.SetFormula("0")
        
        #linkDriverBuilder1.DriverMotions.RotationY.ControlInitialVelocityExpression.SetFormula("0")
        
        #linkDriverBuilder1.DriverMotions.RotationY.ControlInitialAccelerationExpression.SetFormula("0")
        
        linkDriverBuilder1.DriverMotions.RotationZ.DisplacementExpression.SetFormula("0")
        
        linkDriverBuilder1.DriverMotions.RotationZ.VelocityExpression.SetFormula("0")
        
        linkDriverBuilder1.DriverMotions.RotationZ.AccelerationExpression.SetFormula("0")
        
        linkDriverBuilder1.DriverMotions.RotationZ.JerkExpression.SetFormula("0")
        
        linkDriverBuilder1.DriverMotions.RotationZ.AmplitudeExpression.SetFormula("0")
        
        linkDriverBuilder1.DriverMotions.RotationZ.FrequencyExpression.SetFormula("0")
        
        linkDriverBuilder1.DriverMotions.RotationZ.PhaseAngleExpression.SetFormula("0")
        
        linkDriverBuilder1.DriverMotions.RotationZ.HarmonicDisplacementExpression.SetFormula("0")
        
        linkDriverBuilder1.DriverMotions.RotationZ.InitialDisplacementExpression.SetFormula("0")
        
        linkDriverBuilder1.DriverMotions.RotationZ.InitialVelocityExpression.SetFormula("0")
        
        #linkDriverBuilder1.DriverMotions.RotationZ.ControlInitialVelocityExpression.SetFormula("0")
        
        #linkDriverBuilder1.DriverMotions.RotationZ.ControlInitialAccelerationExpression.SetFormula("0")
        
        linkDriverBuilder1.DriverMotions.TranslationX.DisplacementExpression.SetFormula("0")
        
        linkDriverBuilder1.DriverMotions.TranslationX.VelocityExpression.SetFormula("0")
        
        linkDriverBuilder1.DriverMotions.TranslationX.AccelerationExpression.SetFormula("0")
        
        linkDriverBuilder1.DriverMotions.TranslationX.JerkExpression.SetFormula("0")
        
        linkDriverBuilder1.DriverMotions.TranslationX.AmplitudeExpression.SetFormula("0")
        
        linkDriverBuilder1.DriverMotions.TranslationX.FrequencyExpression.SetFormula("0")
        
        linkDriverBuilder1.DriverMotions.TranslationX.PhaseAngleExpression.SetFormula("0")
        
        linkDriverBuilder1.DriverMotions.TranslationX.HarmonicDisplacementExpression.SetFormula("0")
        
        linkDriverBuilder1.DriverMotions.TranslationX.InitialDisplacementExpression.SetFormula("0")
        
        linkDriverBuilder1.DriverMotions.TranslationX.InitialVelocityExpression.SetFormula("0")
        
        #linkDriverBuilder1.DriverMotions.TranslationX.ControlInitialVelocityExpression.SetFormula("0")
        
        #linkDriverBuilder1.DriverMotions.TranslationX.ControlInitialAccelerationExpression.SetFormula("0")
        
        linkDriverBuilder1.DriverMotions.TranslationY.DisplacementExpression.SetFormula("0")
        
        linkDriverBuilder1.DriverMotions.TranslationY.VelocityExpression.SetFormula("0")
        
        linkDriverBuilder1.DriverMotions.TranslationY.AccelerationExpression.SetFormula("0")
        
        linkDriverBuilder1.DriverMotions.TranslationY.JerkExpression.SetFormula("0")
        
        linkDriverBuilder1.DriverMotions.TranslationY.AmplitudeExpression.SetFormula("0")
        
        linkDriverBuilder1.DriverMotions.TranslationY.FrequencyExpression.SetFormula("0")
        
        linkDriverBuilder1.DriverMotions.TranslationY.PhaseAngleExpression.SetFormula("0")
        
        linkDriverBuilder1.DriverMotions.TranslationY.HarmonicDisplacementExpression.SetFormula("0")
        
        linkDriverBuilder1.DriverMotions.TranslationY.InitialDisplacementExpression.SetFormula("0")
        
        linkDriverBuilder1.DriverMotions.TranslationY.InitialVelocityExpression.SetFormula("0")
        
        #linkDriverBuilder1.DriverMotions.TranslationY.ControlInitialVelocityExpression.SetFormula("0")
        
        #linkDriverBuilder1.DriverMotions.TranslationY.ControlInitialAccelerationExpression.SetFormula("0")
        
        linkDriverBuilder1.DriverMotions.TranslationZ.DisplacementExpression.SetFormula("0")
        
        linkDriverBuilder1.DriverMotions.TranslationZ.VelocityExpression.SetFormula("0")
        
        linkDriverBuilder1.DriverMotions.TranslationZ.AccelerationExpression.SetFormula("0")
        
        linkDriverBuilder1.DriverMotions.TranslationZ.JerkExpression.SetFormula("0")
        
        linkDriverBuilder1.DriverMotions.TranslationZ.AmplitudeExpression.SetFormula("0")
        
        linkDriverBuilder1.DriverMotions.TranslationZ.FrequencyExpression.SetFormula("0")
        
        linkDriverBuilder1.DriverMotions.TranslationZ.PhaseAngleExpression.SetFormula("0")
        
        linkDriverBuilder1.DriverMotions.TranslationZ.HarmonicDisplacementExpression.SetFormula("0")
        
        linkDriverBuilder1.DriverMotions.TranslationZ.InitialDisplacementExpression.SetFormula("0")
        
        linkDriverBuilder1.DriverMotions.TranslationZ.InitialVelocityExpression.SetFormula("0")
        
        #linkDriverBuilder1.DriverMotions.TranslationZ.ControlInitialVelocityExpression.SetFormula("0")
        
        #linkDriverBuilder1.DriverMotions.TranslationZ.ControlInitialAccelerationExpression.SetFormula("0")
        
        jointDriverBuilder1.DriverMultiOperations.MotionEulerAngle1.Function = NXOpen.NXObject.Null
        
        jointDriverBuilder1.DriverMultiOperations.MotionEulerAngle1.Function = NXOpen.NXObject.Null
        
        jointDriverBuilder1.DriverMultiOperations.MotionTranslationZ.Function = NXOpen.NXObject.Null
        
        jointDriverBuilder1.DriverMultiOperations.MotionTranslationZ.Function = NXOpen.NXObject.Null
        
        jointDriverBuilder1.DriverMultiOperations.MotionPointOnCurve.Function = NXOpen.NXObject.Null
        
        jointDriverBuilder1.DriverMultiOperations.MotionPointOnCurve.Function = NXOpen.NXObject.Null
        
        expression17 = workPart.Expressions.CreateSystemExpressionWithUnits("0", unit1)
        
        expression18 = workPart.Expressions.CreateSystemExpressionWithUnits("0", unit1)
        
        linkDriverBuilder1.DriverMotions.RotationX.Function = NXOpen.NXObject.Null
        
        linkDriverBuilder1.DriverMotions.RotationX.Function = NXOpen.NXObject.Null
        
        linkDriverBuilder1.DriverMotions.RotationY.Function = NXOpen.NXObject.Null
        
        linkDriverBuilder1.DriverMotions.RotationY.Function = NXOpen.NXObject.Null
        
        linkDriverBuilder1.DriverMotions.RotationZ.Function = NXOpen.NXObject.Null
        
        linkDriverBuilder1.DriverMotions.RotationZ.Function = NXOpen.NXObject.Null
        
        linkDriverBuilder1.DriverMotions.TranslationX.Function = NXOpen.NXObject.Null
        
        linkDriverBuilder1.DriverMotions.TranslationX.Function = NXOpen.NXObject.Null
        
        linkDriverBuilder1.DriverMotions.TranslationY.Function = NXOpen.NXObject.Null
        
        linkDriverBuilder1.DriverMotions.TranslationY.Function = NXOpen.NXObject.Null
        
        linkDriverBuilder1.DriverMotions.TranslationZ.Function = NXOpen.NXObject.Null
        
        linkDriverBuilder1.DriverMotions.TranslationZ.Function = NXOpen.NXObject.Null
        
        globalSelectionBuilder18 = theSession.MotionSession.MotionMethods.GetGlobalSelectionBuilder(workPart)
        
        selectTaggedObjectList17 = globalSelectionBuilder18.Selection
        
        joint1 = self.nxObjectJoints[indexOfDriver] # Appointing driver joint.
        jointDriverBuilder1.Joint.Value = joint1
        self.driver = joint1
        
        
        globalSelectionBuilder19 = theSession.MotionSession.MotionMethods.GetGlobalSelectionBuilder(workPart)
        
        selectTaggedObjectList18 = globalSelectionBuilder19.Selection
        
        
        nXObject8 = jointDriverBuilder1.Commit()
        
        jointDriverBuilder1.Destroy()
        
        linkDriverBuilder1.Destroy()
        
        workPart.MeasureManager.SetPartTransientModification()
        
        workPart.Expressions.Delete(expression17)
        
        workPart.MeasureManager.ClearPartTransientModification()
        
        workPart.MeasureManager.SetPartTransientModification()
        
        workPart.Expressions.Delete(expression18)
        
        workPart.MeasureManager.ClearPartTransientModification()
        
        theSession.CleanUpFacetedFacesAndEdges()
    
    '''
    firstIndex - the index of the first gear in the coupler
    secondIndex - the index of the second gear in the coupler
    '''
    def defineGearCoupler(self, firstIndex, secondIndex):
        #Basic references
        theSession  = NXOpen.Session.GetSession()
        workPart = theSession.Parts.Work
        unit1 = self.unit1
        
        couplerGearBuilder1 = workPart.MotionManager.Couplers.CreateCouplerGearBuilder(NXOpen.Motion.CouplerGear.Null)
        couplerGearBuilder1.RatioExpression.SetFormula("1")
        
        #couplerGearBuilder1.FirstRadiusExpression.SetFormula("50")
        
        #couplerGearBuilder1.SecondRadiusExpression.SetFormula("25")
        
        couplerGearBuilder1.DisplayScale = 1.0

        
        globalSelectionBuilder20 = theSession.MotionSession.MotionMethods.GetGlobalSelectionBuilder(workPart)
        
        selectTaggedObjectList19 = globalSelectionBuilder20.Selection
        
        couplerGearBuilder1.FirstJoint.Value = self.nxObjectJoints[firstIndex] #self.driver # joint1
        
        globalSelectionBuilder21 = theSession.MotionSession.MotionMethods.GetGlobalSelectionBuilder(workPart)
        
        selectTaggedObjectList20 = globalSelectionBuilder21.Selection
        
        globalSelectionBuilder22 = theSession.MotionSession.MotionMethods.GetGlobalSelectionBuilder(workPart)
        
        selectTaggedObjectList21 = globalSelectionBuilder22.Selection
        
        joint2 = self.nxObjectJoints[secondIndex] # nXObject7
        couplerGearBuilder1.SecondJoint.Value = joint2
        
        coordinates1 = NXOpen.Point3d(0.0, 0.0, 10.0)
        point7 = workPart.Points.CreatePoint(coordinates1)
        
        couplerGearBuilder1.FirstRadiusExpression.SetFormula(str(self.bodies[firstIndex].diameter/2)) #("100")

        
        globalSelectionBuilder23 = theSession.MotionSession.MotionMethods.GetGlobalSelectionBuilder(workPart)
        
        selectTaggedObjectList22 = globalSelectionBuilder23.Selection
        
        couplerGearBuilder1.SecondRadiusExpression.SetFormula(str(self.bodies[secondIndex].diameter/2))#("25")

        coordinates2 = NXOpen.Point3d(0.0, 0.0, 10.0)
        point8 = workPart.Points.CreatePoint(coordinates2)
        
        nXObject9 = couplerGearBuilder1.Commit()
        
        couplerGearBuilder1.Destroy()        
        theSession.CleanUpFacetedFacesAndEdges()
        
        
        
    def solve(self, time):
        theSession  = NXOpen.Session.GetSession()
        workPart = theSession.Parts.Work
    
        motionSolutionBuilder1 = workPart.MotionManager.MotionSolutions.CreateSolutionBuilder(NXOpen.Motion.MotionSolution.Null)
                
        unit2 = workPart.UnitCollection.FindObject("Second")
        motionSolutionBuilder1.SetScalarExpressionPropertyValue("SolutionEndTime", str(time), unit2)
        nXObject10 = motionSolutionBuilder1.Commit()

        motionSolutionBuilder1.Destroy()
        
        workPart.MeasureManager.SetPartTransientModification()
        
        workPart.MeasureManager.ClearPartTransientModification()
        
        theSession.MotionSession.MotionMethods.ModelCheck(False)
        
        theSession.CleanUpFacetedFacesAndEdges()
        
        motionSolution1 = nXObject10
        motionSolution1.SolveNormalRunSolution()
        self.myMotionSolution = motionSolution1 #declaring an attribute and assigning it the motion solution
    
    def animate(self):
        theSession  = NXOpen.Session.GetSession()
        
        animationControl1 = self.myMotionSolution.GetAnimationControl()
        theSession.MotionSession.PostProcess.SetInterferenceOption(False)
        theSession.MotionSession.PostProcess.SetMeasureOption(False)
        theSession.MotionSession.PostProcess.SetTraceOption(False)
        theSession.MotionSession.PostProcess.SetStopOnEventOption(False)
        theSession.MotionSession.PostProcess.SetInterferenceOption(False)
        theSession.MotionSession.PostProcess.SetMeasureOption(False)
        theSession.MotionSession.PostProcess.SetTraceOption(False)
        theSession.MotionSession.PostProcess.SetStopOnEventOption(False)
        animationControl1.Delay = 0
        animationControl1.Mode = NXOpen.Motion.PlayMode.Retrace
        animationControl1.StepToAssemblyPosition()
        animationControl1.Play()
        
        animationControl1.StepToAssemblyPosition()
        
        #animationControl1.Finish() - commented out to animate immediately.
        
        theSession.CleanUpFacetedFacesAndEdges()
    
    def initForNX(self):
        #Getting 'fileName' to provide a proper reference to the model.
        #Made based on Save As... journal
        
        pathToTheFolder = self.folder
        fileName = self.fileName

        #Basic references
        theSession  = self.theSession 
        workPart = self.workPart
        displayPart = self.displayPart
        
        #Saving prt file, so now we will know and have a control over the name of the model.
        partSaveStatus1 = workPart.SaveAs(pathToTheFolder + fileName + ".prt")
        partSaveStatus1.Dispose()
        
        #Starting Motion application
        theSession.ApplicationSwitchImmediate("UG_APP_MECHANISMS")
        globalSelectionBuilder1 = theSession.MotionSession.MotionMethods.GetGlobalSelectionBuilder(workPart)
        baseTemplateManager1 = theSession.XYPlotManager.TemplateManager
        
        #Starting New Simulation
        fileNew1 = theSession.Parts.FileNew()
        fileNew1.TemplateFileName = "Simcenter 3D Motion"
        fileNew1.UseBlankTemplate = True
        fileNew1.ApplicationName = "MotionTemplate"
        fileNew1.Units = NXOpen.Part.Units.Millimeters
        fileNew1.RelationType = ""
        fileNew1.UsesMasterModel = "Yes"
        fileNew1.TemplateType = NXOpen.FileNewTemplateType.Item
        fileNew1.TemplatePresentationName = ""
        fileNew1.ItemType = ""
        fileNew1.Specialization = ""
        fileNew1.SetCanCreateAltrep(False)
        
        #NB! Here is the place to provide a proper (=unique) name to the sim(ulation) file.
        fileNew1.NewFileName = pathToTheFolder + fileName + "_motion1_"+ str(random.randint(1,10000)) + ".sim"
        #NB! And here is the place to provide that known (controled) name of the model.
        fileNew1.MasterFileName = fileName # was set above, in the beginning of this initForNX() method.
        
        fileNew1.MakeDisplayedPart = True
        fileNew1.DisplayPartOption = NXOpen.DisplayPartOption.AllowAdditional
        baseTemplateManager2 = theSession.XYPlotManager.TemplateManager
        #First NX Object to comit the simulation model.
        nXObject1 = fileNew1.Commit()
        workPart = theSession.Parts.Work # The motion model
        displayPart = theSession.Parts.Display # The motion model
        
        #Starting the Motion application
        theSession.MotionSession.Environments.SetAnalysisType(NXOpen.Motion.MotionEnvironment.Analysis.Dynamics)
        theSession.MotionSession.Environments.SetComponentBasedMechanism(False)
        theSession.MotionSession.Environments.SetJointWizardStatus(NXOpen.Motion.MotionEnvironment.JointWizardStatus.On)
        theSession.MotionSession.InitializeSimulation(workPart)

        theSession.MotionSession.MotionMethods.ModelCheck(False)
        fileNew1.Destroy()
        
        #Adding the Motion Bodies
        #Support elements required through out the code
        # Unitis definition
        unit1 = workPart.UnitCollection.FindObject("MilliMeter")
        self.unit1 = unit1
        # The component that is used to search for NX Objects
        component1 = workPart.ComponentAssembly.RootComponent.FindObject("COMPONENT " + self.fileName + " 1")
        self.component1 = component1 #Creating a reference (Motion2 class attribute) to that component1 to use it in methods.
        self.defineMotionBodies(self.bodies)
        
        # ----------------------------------------------
        #   Joints defintion
        # ----------------------------------------------
        self.defineJoints(self.bodies)

        
        # ----------------------------------------------
        #   Menu: Insert->Driver->Driver...
        # ----------------------------------------------
        # Let it be first joint - index = 0, vel=10 deg/sec, accel = 0, dir = True
        self.defineDriver(0, 10, 0, True)
        
        
        # ----------------------------------------------
        #   Menu: Insert->Coupler->Gear Coupler...
        # ----------------------------------------------
        #self.defineGearCoupler(0,1)
        #Make a for loop to take pairs of joints and make gear couplings
        for i in range(0, len(self.nxObjectJoints)-1):
            self.defineGearCoupler(i,i+1)
            
        
        # ----------------------------------------------
        #   Dialog Begin Solution
        # ----------------------------------------------
        self.solve(self.simTime)
        
        # ----------------------------------------------
        #   Menu: Analysis->Motion->Animation Player...
        # ----------------------------------------------
        self.animate()
        
        
    
    
    

