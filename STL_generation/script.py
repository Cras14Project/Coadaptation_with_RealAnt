import FreeCAD
from FreeCAD import Base, Vector
import Part
from math import pi, sin, cos
import Mesh
import PartDesignGui
import PartGui
import MeshPart
import Show
import Sketcher
import os


#Edit these parameters:
# length of the leg
length = 50	
# radius of the ronded tip of the leg
tip_radius = 3

#path to current directory
dir_path = os.path.dirname(os.path.realpath(__file__))
# location of the original leg model
orig_file = dir_path + "\leg_v8.stl"
# location where the new generated file should be saved
new_file = dir_path + "\\new_leg.stl"


Mesh.open(orig_file)
FreeCAD.setActiveDocument("Unnamed")
FreeCAD.getDocument('Unnamed').Label = "Leg"
DOC = FreeCAD.activeDocument()


def setview():
    """Rearrange View"""
    FreeCAD.Gui.SendMsgToActiveView("ViewFit")
    FreeCAD.Gui.activeDocument().activeView().viewAxometric()

def convert_to_part():
	"""Converts the imported mesh model to solid part"""
	### Begin command Part_ShapeFromMesh
	FreeCAD.getDocument('Unnamed').addObject('Part::Feature', 'leg_v8_shape')
	__shape__ = Part.Shape()
	__shape__.makeShapeFromMesh(FreeCAD.getDocument('Unnamed').getObject('leg_v8').Mesh.Topology, 0.100000, False)
	FreeCAD.getDocument('Unnamed').getObject('leg_v8_shape').Shape = __shape__
	FreeCAD.getDocument('Unnamed').getObject('leg_v8_shape').purgeTouched()
	del __shape__
	### End command Part_ShapeFromMesh

	FreeCAD.Gui.Selection.addSelection('Unnamed','leg_v8')
	FreeCAD.Gui.runCommand('Std_ToggleVisibility',0)
	FreeCAD.Gui.Selection.clearSelection()
	FreeCAD.Gui.Selection.addSelection('Unnamed','leg_v8_shape')
	
	### Begin command Part_MakeSolid
	__s__=FreeCAD.ActiveDocument.leg_v8_shape.Shape.Faces
	__s__=Part.Solid(Part.Shell(__s__))
	__o__=FreeCAD.ActiveDocument.addObject("Part::Feature","leg_v8_solid")
	__o__.Label="leg_v8__2_001 (Solid)"
	__o__.Shape=__s__
	del __s__, __o__
	### End command Part_MakeSolid
	
	FreeCAD.Gui.runCommand('Std_ToggleVisibility',0)
	FreeCAD.Gui.Selection.clearSelection()
	FreeCAD.Gui.Selection.addSelection('Unnamed','leg_v8_solid')

	### Begin command Part_RefineShape
	FreeCAD.ActiveDocument.addObject('Part::Refine','leg_v8_solid_refined').Source=App.ActiveDocument.leg_v8_solid
	FreeCAD.ActiveDocument.ActiveObject.Label=App.ActiveDocument.leg_v8_solid.Label
	FreeCAD.Gui.ActiveDocument.leg_v8_solid.hide()
	FreeCAD.ActiveDocument.recompute()
	### End command Part_RefineShape

	FreeCAD.Gui.Selection.clearSelection()
	FreeCAD.Gui.Selection.addSelection('Unnamed','leg_v8_solid_refined')
	FreeCAD.Gui.runCommand('Std_ToggleVisibility',0)
	
	### Begin command PartDesign_Body
	FreeCAD.activeDocument().addObject('PartDesign::Body','Body')
	FreeCAD.activeDocument().Body.BaseFeature = App.activeDocument().leg_v8_solid_refined
	FreeCAD.Gui.activateView('Gui::View3DInventor', True)
	FreeCAD.Gui.activeView().setActiveObject('pdbody', App.activeDocument().Body)
	FreeCAD.Gui.Selection.clearSelection()
	FreeCAD.Gui.Selection.addSelection(App.ActiveDocument.Body)
	FreeCAD.ActiveDocument.recompute()
	FreeCAD.Gui.Selection.clearSelection()
	### End command PartDesign_Body

#Sketches a rectangle so that part of the original model can be removed
def sketch():
	"""Sketches an area to be removed from the original model so that new leg design can be created instead"""
	FreeCAD.Gui.activateWorkbench("PartDesignWorkbench")
	### Begin command PartDesign_NewSketch
	FreeCAD.getDocument('Unnamed').getObject('Body').newObject('Sketcher::SketchObject','Sketch')
	FreeCAD.getDocument('Unnamed').getObject('Sketch').Support = (FreeCAD.getDocument('Unnamed').getObject('BaseFeature'),['Face82',])
	FreeCAD.getDocument('Unnamed').getObject('Sketch').MapMode = 'FlatFace'
	FreeCAD.ActiveDocument.recompute()
	FreeCAD.Gui.getDocument('Unnamed').setEdit(FreeCAD.getDocument('Unnamed').getObject('Body'), 0, 'Sketch.')
	ActiveSketch = FreeCAD.getDocument('Unnamed').getObject('Sketch')
	tv = Show.TempoVis(FreeCAD.ActiveDocument, tag= ActiveSketch.ViewObject.TypeId)
	ActiveSketch.ViewObject.TempoVis = tv
	if ActiveSketch.ViewObject.EditingWorkbench:
		tv.activateWorkbench(ActiveSketch.ViewObject.EditingWorkbench)
	if ActiveSketch.ViewObject.HideDependent:
		tv.hide(tv.get_all_dependent(FreeCAD.getDocument('Unnamed').getObject('Body'), 'Sketch.'))
	if ActiveSketch.ViewObject.ShowSupport:
		tv.show([ref[0] for ref in ActiveSketch.Support if not ref[0].isDerivedFrom("PartDesign::Plane")])
	if ActiveSketch.ViewObject.ShowLinks:
		tv.show([ref[0] for ref in ActiveSketch.ExternalGeometry])
	tv.sketchClipPlane(ActiveSketch, ActiveSketch.ViewObject.SectionView)
	tv.hide(ActiveSketch)
	del(tv)
	del(ActiveSketch)
	
	ActiveSketch = FreeCAD.getDocument('Unnamed').getObject('Sketch')
	if ActiveSketch.ViewObject.RestoreCamera:
		ActiveSketch.ViewObject.TempoVis.saveCamera()
		if ActiveSketch.ViewObject.ForceOrtho:
			ActiveSketch.ViewObject.Document.ActiveView.setCameraType('Orthographic')
	### End command PartDesign_NewSketch
	FreeCAD.Gui.runCommand('Sketcher_CompCreateRectangles',0)
	geoList = []
	geoList.append(Part.LineSegment(Vector(-247.146866,13.465923,0),Vector(-171.100571,13.465923,0)))
	geoList.append(Part.LineSegment(Vector(-171.100571,13.465923,0),Vector(-171.100571,-45.323715,0)))
	geoList.append(Part.LineSegment(Vector(-171.100571,-45.323715,0),Vector(-247.146866,-45.323715,0)))
	geoList.append(Part.LineSegment(Vector(-247.146866,-45.323715,0),Vector(-247.146866,13.465923,0)))
	FreeCAD.getDocument('Unnamed').getObject('Sketch').addGeometry(geoList,False)
	conList = []
	conList.append(Sketcher.Constraint('Coincident',0,2,1,1))
	conList.append(Sketcher.Constraint('Coincident',1,2,2,1))
	conList.append(Sketcher.Constraint('Coincident',2,2,3,1))
	conList.append(Sketcher.Constraint('Coincident',3,2,0,1))
	conList.append(Sketcher.Constraint('Horizontal',0))
	conList.append(Sketcher.Constraint('Horizontal',2))
	conList.append(Sketcher.Constraint('Vertical',1))
	conList.append(Sketcher.Constraint('Vertical',3))
	FreeCAD.getDocument('Unnamed').getObject('Sketch').addConstraint(conList)
	del geoList, conList

#Cuts sketched area out
def cut_part():
	"""Removes part of the original leg defined by sketch made by sketch()"""
	FreeCAD.Gui.getDocument('Unnamed').resetEdit()
	FreeCAD.ActiveDocument.recompute()
	ActiveSketch = App.getDocument('Unnamed').getObject('Sketch')
	tv = ActiveSketch.ViewObject.TempoVis
	if tv:
		tv.restore()
	ActiveSketch.ViewObject.TempoVis = None
	del(tv)
	del(ActiveSketch)

	FreeCAD.Gui.Selection.addSelection('Unnamed','Body','Sketch.')
	FreeCAD.getDocument('Unnamed').recompute()
	### Begin command PartDesign_Pocket
	FreeCAD.getDocument('Unnamed').getObject('Body').newObject('PartDesign::Pocket','Pocket')
	FreeCAD.getDocument('Unnamed').getObject('Pocket').Profile = FreeCAD.getDocument('Unnamed').getObject('Sketch')
	FreeCAD.getDocument('Unnamed').getObject('Pocket').Length = 5
	FreeCAD.ActiveDocument.recompute()
	FreeCAD.getDocument('Unnamed').getObject('Pocket').ReferenceAxis = (FreeCAD.getDocument('Unnamed').getObject('Sketch'),['N_Axis'])
	FreeCAD.getDocument('Unnamed').getObject('Sketch').Visibility = False
	FreeCAD.ActiveDocument.recompute()
	FreeCAD.getDocument('Unnamed').getObject('Pocket').ViewObject.ShapeColor=getattr(FreeCAD.getDocument('Unnamed').getObject('BaseFeature').getLinkedObject(True).ViewObject,'ShapeColor',FreeCAD.getDocument('Unnamed').getObject('Pocket').ViewObject.ShapeColor)
	FreeCAD.getDocument('Unnamed').getObject('Pocket').ViewObject.LineColor=getattr(FreeCAD.getDocument('Unnamed').getObject('BaseFeature').getLinkedObject(True).ViewObject,'LineColor',FreeCAD.getDocument('Unnamed').getObject('Pocket').ViewObject.LineColor)
	FreeCAD.getDocument('Unnamed').getObject('Pocket').ViewObject.PointColor=getattr(FreeCAD.getDocument('Unnamed').getObject('BaseFeature').getLinkedObject(True).ViewObject,'PointColor',FreeCAD.getDocument('Unnamed').getObject('Pocket').ViewObject.PointColor)
	FreeCAD.getDocument('Unnamed').getObject('Pocket').ViewObject.Transparency=getattr(FreeCAD.getDocument('Unnamed').getObject('BaseFeature').getLinkedObject(True).ViewObject,'Transparency',FreeCAD.getDocument('Unnamed').getObject('Pocket').ViewObject.Transparency)
	FreeCAD.getDocument('Unnamed').getObject('Pocket').ViewObject.DisplayMode=getattr(FreeCAD.getDocument('Unnamed').getObject('BaseFeature').getLinkedObject(True).ViewObject,'DisplayMode',FreeCAD.getDocument('Unnamed').getObject('Pocket').ViewObject.DisplayMode)
	FreeCAD.Gui.getDocument('Unnamed').setEdit(FreeCAD.getDocument('Unnamed').getObject('Body'), 0, 'Pocket.')
	### End command PartDesign_Pocket
	FreeCAD.Gui.Selection.clearSelection()
	FreeCAD.getDocument('Unnamed').getObject('Pocket').UseCustomVector = 0
	FreeCAD.getDocument('Unnamed').getObject('Pocket').Direction = (0, 1, -0)
	FreeCAD.getDocument('Unnamed').getObject('Pocket').ReferenceAxis = (App.getDocument('Unnamed').getObject('Sketch'), ['N_Axis'])
	FreeCAD.getDocument('Unnamed').getObject('Pocket').AlongSketchNormal = 1
	FreeCAD.getDocument('Unnamed').getObject('Pocket').Type = 1
	FreeCAD.getDocument('Unnamed').getObject('Pocket').UpToFace = None
	FreeCAD.getDocument('Unnamed').getObject('Pocket').Reversed = 0
	FreeCAD.getDocument('Unnamed').getObject('Pocket').Midplane = 0
	FreeCAD.getDocument('Unnamed').getObject('Pocket').Offset = 0
	FreeCAD.getDocument('Unnamed').recompute()
	FreeCAD.getDocument('Unnamed').getObject('BaseFeature').Visibility = False
	FreeCAD.Gui.getDocument('Unnamed').resetEdit()
	FreeCAD.getDocument('Unnamed').getObject('Sketch').Visibility = False

def sketch_pad(length_):
	"""Sketches triangle to be padded. Parameter length_ defines the length of the leg."""
	FreeCAD.Gui.activateWorkbench("PartDesignWorkbench")
	FreeCAD.getDocument('Unnamed').getObject('Body').newObject('Sketcher::SketchObject','Sketch001')
	FreeCAD.getDocument('Unnamed').getObject('Sketch001').Support = (FreeCAD.getDocument('Unnamed').getObject('XZ_Plane'),[''])
	FreeCAD.getDocument('Unnamed').getObject('Sketch001').MapMode = 'FlatFace'
	FreeCAD.ActiveDocument.recompute()
	FreeCAD.Gui.getDocument('Unnamed').setEdit(FreeCAD.getDocument('Unnamed').getObject('Body'), 0, 'Sketch001.')
	ActiveSketch = FreeCAD.getDocument('Unnamed').getObject('Sketch001')
	tv = Show.TempoVis(FreeCAD.ActiveDocument, tag= ActiveSketch.ViewObject.TypeId)
	ActiveSketch.ViewObject.TempoVis = tv
	if ActiveSketch.ViewObject.EditingWorkbench:
		tv.activateWorkbench(ActiveSketch.ViewObject.EditingWorkbench)
	if ActiveSketch.ViewObject.HideDependent:
		tv.hide(tv.get_all_dependent(FreeCAD.getDocument('Unnamed').getObject('Body'), 'Sketch001.'))
	if ActiveSketch.ViewObject.ShowSupport:
		tv.show([ref[0] for ref in ActiveSketch.Support if not ref[0].isDerivedFrom("PartDesign::Plane")])
	if ActiveSketch.ViewObject.ShowLinks:
		tv.show([ref[0] for ref in ActiveSketch.ExternalGeometry])
	tv.sketchClipPlane(ActiveSketch, ActiveSketch.ViewObject.SectionView)
	tv.hide(ActiveSketch)
	del(tv)
	del(ActiveSketch)
 
	ActiveSketch = FreeCAD.getDocument('Unnamed').getObject('Sketch001')
	if ActiveSketch.ViewObject.RestoreCamera:
		ActiveSketch.ViewObject.TempoVis.saveCamera()
		if ActiveSketch.ViewObject.ForceOrtho:
			ActiveSketch.ViewObject.Document.ActiveView.setCameraType('Orthographic')

	FreeCAD.Gui.Selection.clearSelection()
	FreeCAD.Gui.runCommand('Sketcher_CreateLine',0)
	FreeCAD.Gui.runCommand('Sketcher_External',0)
	FreeCAD.Gui.Selection.addSelection('Unnamed','Body','Pocket.Edge19',-171.101,-5,-11.258)
	FreeCAD.Gui.Selection.clearSelection()
	FreeCAD.Gui.runCommand('Sketcher_CreateLine',0)
	FreeCAD.getDocument('Unnamed').getObject('Sketch001').addGeometry(Part.LineSegment(Vector(-171.100571,-2.970799,0),Vector(-171.100571-length_,-19,0)),False)
	FreeCAD.getDocument('Unnamed').getObject('Sketch001').addConstraint(Sketcher.Constraint('Coincident',0,1,-3,1)) 
	FreeCAD.getDocument('Unnamed').getObject('Sketch001').addGeometry(Part.LineSegment(Vector(-171.100571,-35.027782,0),Vector(-171.100571-length_,-19,0)),False)
	FreeCAD.getDocument('Unnamed').getObject('Sketch001').addConstraint(Sketcher.Constraint('Coincident',1,1,-3,2)) 
	FreeCAD.getDocument('Unnamed').getObject('Sketch001').addConstraint(Sketcher.Constraint('Coincident',1,2,0,2)) 
	FreeCAD.getDocument('Unnamed').getObject('Sketch001').addGeometry(Part.LineSegment(Vector(-171.100571,-2.970799,0),Vector(-171.100571,-35.027782,0)),False)
	FreeCAD.getDocument('Unnamed').getObject('Sketch001').addConstraint(Sketcher.Constraint('Coincident',2,1,0,1)) 
	FreeCAD.getDocument('Unnamed').getObject('Sketch001').addConstraint(Sketcher.Constraint('Coincident',2,2,1,1)) 
	FreeCAD.Gui.getDocument('Unnamed').resetEdit()
	FreeCAD.ActiveDocument.recompute()
	ActiveSketch = FreeCAD.getDocument('Unnamed').getObject('Sketch001')
	tv = ActiveSketch.ViewObject.TempoVis
	if tv:
		tv.restore()
	ActiveSketch.ViewObject.TempoVis = None
	del(tv)
	del(ActiveSketch)

def pad():
	"""Pads the sketched area."""
	### Begin command PartDesign_Pad
	FreeCAD.Gui.Selection.addSelection('Unnamed','Body','Sketch001.')
	FreeCAD.getDocument('Unnamed').getObject('Body').newObject('PartDesign::Pad','Pad')
	FreeCAD.getDocument('Unnamed').getObject('Pad').Profile = FreeCAD.getDocument('Unnamed').getObject('Sketch001')
	FreeCAD.getDocument('Unnamed').getObject('Pad').Length = 10
	FreeCAD.ActiveDocument.recompute()
	FreeCAD.getDocument('Unnamed').getObject('Sketch001').Visibility = False
	FreeCAD.ActiveDocument.recompute()
	FreeCAD.getDocument('Unnamed').getObject('Pad').ViewObject.ShapeColor=getattr(FreeCAD.getDocument('Unnamed').getObject('Pocket').getLinkedObject(True).ViewObject,'ShapeColor',FreeCAD.getDocument('Unnamed').getObject('Pad').ViewObject.ShapeColor)
	FreeCAD.getDocument('Unnamed').getObject('Pad').ViewObject.LineColor=getattr(FreeCAD.getDocument('Unnamed').getObject('Pocket').getLinkedObject(True).ViewObject,'LineColor',FreeCAD.getDocument('Unnamed').getObject('Pad').ViewObject.LineColor)
	FreeCAD.getDocument('Unnamed').getObject('Pad').ViewObject.PointColor=getattr(FreeCAD.getDocument('Unnamed').getObject('Pocket').getLinkedObject(True).ViewObject,'PointColor',FreeCAD.getDocument('Unnamed').getObject('Pad').ViewObject.PointColor)
	FreeCAD.getDocument('Unnamed').getObject('Pad').ViewObject.Transparency=getattr(FreeCAD.getDocument('Unnamed').getObject('Pocket').getLinkedObject(True).ViewObject,'Transparency',FreeCAD.getDocument('Unnamed').getObject('Pad').ViewObject.Transparency)
	FreeCAD.getDocument('Unnamed').getObject('Pad').ViewObject.DisplayMode=getattr(FreeCAD.getDocument('Unnamed').getObject('Pocket').getLinkedObject(True).ViewObject,'DisplayMode',FreeCAD.getDocument('Unnamed').getObject('Pad').ViewObject.DisplayMode)
	FreeCAD.Gui.getDocument('Unnamed').setEdit(FreeCAD.getDocument('Unnamed').getObject('Body'), 0, 'Pad.')
	FreeCAD.Gui.Selection.clearSelection()
	### End command PartDesign_Pad

	FreeCAD.getDocument('Unnamed').getObject('Pad').Length = 10.000000
	FreeCAD.getDocument('Unnamed').getObject('Pad').TaperAngle = 0.000000
	FreeCAD.getDocument('Unnamed').getObject('Pad').UseCustomVector = 0
	FreeCAD.getDocument('Unnamed').getObject('Pad').Direction = (0, -1, 0)
	FreeCAD.getDocument('Unnamed').getObject('Pad').ReferenceAxis = (FreeCAD.getDocument('Unnamed').getObject('Sketch001'), ['N_Axis'])
	FreeCAD.getDocument('Unnamed').getObject('Pad').AlongSketchNormal = 1
	FreeCAD.getDocument('Unnamed').getObject('Pad').Type = 0
	FreeCAD.getDocument('Unnamed').getObject('Pad').UpToFace = None
	FreeCAD.getDocument('Unnamed').getObject('Pad').Reversed = 0
	FreeCAD.getDocument('Unnamed').getObject('Pad').Midplane = 1
	FreeCAD.getDocument('Unnamed').getObject('Pad').Offset = 0
	FreeCAD.getDocument('Unnamed').recompute()
	FreeCAD.getDocument('Unnamed').getObject('Pocket').Visibility = False
	FreeCAD.Gui.getDocument('Unnamed').resetEdit()
	FreeCAD.getDocument('Unnamed').getObject('Sketch001').Visibility = False

def round(R):
	"""Rounds the tip of the leg with the circle with radius R millimeters."""
	FreeCAD.Gui.Selection.addSelection('Unnamed','Body','Pad.Edge100',-221.101,-0.958734,-19)
	### Begin command PartDesign_Fillet
	FreeCAD.getDocument('Unnamed').getObject('Body').newObject('PartDesign::Fillet','Fillet')
	FreeCAD.getDocument('Unnamed').getObject('Fillet').Base = (App.getDocument('Unnamed').getObject('Pad'),['Edge100',])
	FreeCAD.Gui.Selection.clearSelection()
	FreeCAD.getDocument('Unnamed').getObject('Pad').Visibility = False
	FreeCAD.ActiveDocument.recompute()
	FreeCAD.getDocument('Unnamed').getObject('Fillet').ViewObject.ShapeColor=getattr(FreeCAD.getDocument('Unnamed').getObject('Pad').getLinkedObject(True).ViewObject,'ShapeColor',FreeCAD.getDocument('Unnamed').getObject('Fillet').ViewObject.ShapeColor)
	FreeCAD.getDocument('Unnamed').getObject('Fillet').ViewObject.LineColor=getattr(FreeCAD.getDocument('Unnamed').getObject('Pad').getLinkedObject(True).ViewObject,'LineColor',FreeCAD.getDocument('Unnamed').getObject('Fillet').ViewObject.LineColor)
	FreeCAD.getDocument('Unnamed').getObject('Fillet').ViewObject.PointColor=getattr(FreeCAD.getDocument('Unnamed').getObject('Pad').getLinkedObject(True).ViewObject,'PointColor',FreeCAD.getDocument('Unnamed').getObject('Fillet').ViewObject.PointColor)
	FreeCAD.getDocument('Unnamed').getObject('Fillet').ViewObject.Transparency=getattr(FreeCAD.getDocument('Unnamed').getObject('Pad').getLinkedObject(True).ViewObject,'Transparency',FreeCAD.getDocument('Unnamed').getObject('Fillet').ViewObject.Transparency)
	FreeCAD.getDocument('Unnamed').getObject('Fillet').ViewObject.DisplayMode=getattr(FreeCAD.getDocument('Unnamed').getObject('Pad').getLinkedObject(True).ViewObject,'DisplayMode',FreeCAD.getDocument('Unnamed').getObject('Fillet').ViewObject.DisplayMode)
	FreeCAD.Gui.getDocument('Unnamed').setEdit(FreeCAD.getDocument('Unnamed').getObject('Body'), 0, 'Fillet.')
	FreeCAD.Gui.Selection.clearSelection()
	### End command PartDesign_Fillet
	FreeCAD.Gui.Selection.clearSelection()
	FreeCAD.getDocument('Unnamed').getObject('Fillet').Radius = R
	FreeCAD.getDocument('Unnamed').getObject('Fillet').Base = (App.getDocument('Unnamed').getObject('Pad'),["Edge100",])
	FreeCAD.getDocument('Unnamed').recompute()
	FreeCAD.getDocument('Unnamed').getObject('Pad').Visibility = False
	FreeCAD.Gui.getDocument('Unnamed').resetEdit()

def convert_to_mesh():
	"""Converts the solid part to mesh model."""
	FreeCAD.Gui.activateWorkbench("MeshWorkbench")
	FreeCAD.Gui.Selection.addSelection('Unnamed','Body')
	 ### Begin command Mesh_FromPartShape
	FreeCADGui.runCommand('MeshPart_Mesher')
	### End command Mesh_FromPartShape
	__doc__=FreeCAD.getDocument("Unnamed")
	__mesh__=__doc__.addObject("Mesh::Feature","Mesh")
	__part__=__doc__.getObject("Body")
	__shape__=Part.getShape(__part__,"")
	__mesh__.Mesh=MeshPart.meshFromShape(Shape=__shape__, LinearDeflection=0.1, AngularDeflection=0.523599, Relative=False)
	__mesh__.Label="Body (Meshed)"
	del __doc__, __mesh__, __part__, __shape__
	FreeCAD.Gui.runCommand('Std_HideSelection',0)
	FreeCAD.Gui.Selection.clearSelection()
	FreeCAD.Gui.Selection.addSelection('Unnamed','Mesh')
	FreeCAD.Gui.runCommand('Std_DrawStyle',6)

def export(location):
	"""Exports the model as STL-file to specified location."""
	doc = FreeCAD.ActiveDocument
	Mesh.export([doc.Mesh], location)

convert_to_part()
setview()
sketch()
cut_part()
sketch_pad(length)
pad()
round(tip_radius)
convert_to_mesh()
export(new_file)


