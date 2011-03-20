import math
from xml.dom import minidom
import xml.etree.ElementTree as ET

from joints import *
from utils import *

def export_ogre_skeleton_xml(anim):
    rootoffset = [0.0, 1.37, 0.0] #TODO: HACK: magic number

    root = ET.Element("skeleton")
    bones = ET.SubElement(root, "bones")
    for i, body in enumerate(anim.Character.BodyList):
	bone = ET.SubElement(bones, "bone")
	bone.set("id", str(i))
	bone.set("name", str(body.Name))

	position = ET.SubElement(bone, "position")
	if i == 0:
	    position.set("x", "%.8f" % rootoffset[0])
	    position.set("y", "%.8f" % rootoffset[1])
	    position.set("z", "%.8f" % rootoffset[2])
	else:
	    offset = [ body.ParentJoint.PointA[i] - body.Parent.ep_a()[i] for i in range(len(body.Parent.ep_a()))]
	    position.set("x", "%.8f" % offset[0])
	    position.set("y", "%.8f" % -offset[1])
	    position.set("z", "%.8f" % offset[2])

	rotation = ET.SubElement(bone, "rotation")
	if i == 0:
	    rotation.set("angle", "%.8f" % math.pi)
	    axis = ET.SubElement(rotation, "axis")
	    axis.set("x", "%.8f" % 0.0)
	    axis.set("y", "%.8f" % 0.0)
	    axis.set("z", "%.8f" % 1.0)
	else:
	    rotation.set("angle", "%.8f" % 0.0)
	    axis = ET.SubElement(rotation, "axis")
	    axis.set("x", "%.8f" % 1.0)
	    axis.set("y", "%.8f" % 0.0)
	    axis.set("z", "%.8f" % 0.0)

    bonehierarchy = ET.SubElement(root, "bonehierarchy")
    for i, joint in enumerate(anim.Character.JointList):
	if isinstance(joint, JointRevolute):
	    boneparent = ET.SubElement(bonehierarchy, "boneparent")
	    boneparent.set("bone", str(joint.BodyB.Name))
	    boneparent.set("parent", str(joint.BodyA.Name))

    animations = ET.SubElement(root, "animations")
    animation = ET.SubElement(animations, "animation")
    animation.set("name", str(anim.Name))
    animation.set("length", str(anim.get_frame_count() * anim.get_frame_length()))
    tracks = ET.SubElement(animation, "tracks")

    for i, body in enumerate(anim.Character.BodyList):
	track = ET.SubElement(tracks, "track")
	track.set("bone", str(body.Name))
	keyframes = ET.SubElement(track, "keyframes")
	for frame in range(anim.get_frame_count()):
	    keyframe = ET.SubElement(keyframes, "keyframe")
	    keyframe.set("time", str(frame * anim.get_frame_length()))

	    bonepos = []
	    if i == 0:
		#get position of root
		q = [anim.SolutionValues[str(body.q[x])][frame] for x in range(dof)]
		bonepos = world_xf(body.ep_a(), q)
		#in ogre, the root translation seems to be expressed relative to
		#its "root pose" specified above, so here we subtract it out
		bonepos[0] = bonepos[0] - rootoffset[0]
		bonepos[1] = bonepos[1] - rootoffset[1]
		bonepos[2] = bonepos[2] - rootoffset[2]
	    else:
		bonepos = [0.0]*3
	    translate = ET.SubElement(keyframe, "translate")
	    translate.set("x", "%.8f" % bonepos[0])
	    translate.set("y", "%.8f" % bonepos[1])
	    translate.set("z", "%.8f" % bonepos[2])

	    #euler = [anim.SolutionValues[str(body.q[x])][frame] for x in range(3,dof)]
	    #axisangle = euler_to_axisangle(euler)
	    #axisangle = map(float, axisangle)
	    axisangle = [0.0]*4
	    if i == 0:
		rootEuler = [anim.SolutionValues[str(body.q[x])][frame] for x in range(3,dof)]
		rootMat = euler_to_matrix(rootEuler).evalf()
		#convert to axis angle... uh, by way of a quat :)
		quat = matrix_to_quat(rootMat)
		axisangle = quat_to_axisangle(quat)
	    else:
		childEuler = [anim.SolutionValues[str(body.q[x])][frame] for x in range(3,dof)]
		childMat = euler_to_matrix(childEuler).evalf()

		parentEuler = [anim.SolutionValues[str(body.Parent.q[x])][frame] for x in range(3,dof)]
		parentMat = euler_to_matrix(parentEuler).evalf()

		mat = (parentMat.inv() * childMat).evalf() #express child relative to parent

		#convert to axis angle... by way of a quat :)
		quat = matrix_to_quat(mat)
		axisangle = quat_to_axisangle(quat)

	    rotate = ET.SubElement(keyframe, "rotate")
	    rotate.set("angle", "%.8f" % axisangle[3])
	    axis = ET.SubElement(rotate, "axis")
	    axis.set("x", "%.8f" % -axisangle[0])
	    axis.set("y", "%.8f" % -axisangle[1])   #TODO: negatives here? Or do it in quat, or axisangle? :S
	    axis.set("z", "%.8f" % axisangle[2])

    #TODO: HACK: oh the lengths I'll go to for some pretty printing
    rough_string = ET.tostring(root, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml()

def _get_bvh_hierarchy(character, root, level, rootoffset):
    ret = ''
    tab = '\t' * level
    if level == 0:
	#special case for root
	ret += '%sROOT %s\n' % (tab, root.Name)
	ret += '%s{\n' % tab
	level += 1; tab = '\t' * level;
	ret += '%sOFFSET\t%f\t%f\t%f\n' % tuple([tab] + rootoffset)
	ret += '%sCHANNELS 6 Xposition Yposition Zposition Xrotation Yrotation Zrotation\n' % tab
    else:
	#regular case
	ret += '%sJOINT %s\n' % (tab, root.Name)
	ret += '%s{\n' % tab
	level += 1; tab = '\t' * level;
	offset = [ root.ParentJoint.PointA[i] - root.Parent.ep_a()[i] for i in range(len(root.Parent.ep_a()))]
	ret += '%sOFFSET\t%f\t%f\t%f\n' % tuple([tab] + offset)
	ret += '%sCHANNELS 3 Xrotation Yrotation Zrotation\n' % tab
    if len(root.ChildList) > 0:
	for child in root.ChildList:
	    ret += _get_bvh_hierarchy(character, child, level, rootoffset)
    else:
	ret += '%sEnd Site\n' % tab
	ret += '%s{\n' % tab
	level += 1; tab = '\t' * level;
	ret += '%sOFFSET\t%f\t%f\t%f\n' % tuple([tab] + [0, -root.Diameter[1], 0])
	level -= 1; tab = '\t' * level;
	ret += '%s}\n' % tab
    level -= 1; tab = '\t' * level;
    ret += '%s}\n' % tab
    return ret

def _get_bvh_motion(character, body, level, frame, data):
    ret = ''
    #special case for root
    if level == 0:
	q = [data[str(body.q[x])][frame] for x in range(dof)]
	pos = world_xf(body.ep_a(), q)
	pos = map(float, pos)	#evaluate sympy expression to a float
	ret += '{: .8f} {: .8f} {: .8f} '.format(*pos)

	rot = [float(q[x] * (180.0 / math.pi)) for x in range(3,dof)] #convert to degrees
	ret += '{: .8f} {: .8f} {: .8f} '.format(*rot)

    #regular case
    else:
	#there *might* be a nicer way to handle relative rotations using just euler angles
	#but matricies are easy! so this will convert them to matricies, express
	#the child relative to the parent, and convert back to euler angles again
	childEuler = [data[str(body.q[x])][frame] for x in range(3,dof)]
	childMat = euler_to_matrix(childEuler).evalf()

	parentEuler = [data[str(body.Parent.q[x])][frame] for x in range(3,dof)]
	parentMat = euler_to_matrix(parentEuler).evalf()

	m = (parentMat.inv() * childMat).evalf() #express child relative to parent
	r = matrix_to_euler(m)
	r = [float(x * (180.0 / math.pi)) for x in r]	#convert to degrees
	ret += '{: .8f} {: .8f} {: .8f} '.format(*r)

    level += 1;
    for child in body.ChildList:
	ret += _get_bvh_motion(character, child, level, frame, data)
    level -= 1;

    return ret

def export_bvh(anim):
    '''Returns a string containing the animation in BVH format'''

    ret = ''
    root = anim.Character.BodyList[0]

    #write hierarchy
    ret += 'HIERARCHY\n'
    ret += _get_bvh_hierarchy(anim.Character, root, 0, [0, 1.37, 0])	#TODO: magic number

    #write motion
    ret += 'MOTION\n'
    ret += 'Frames: %i\n' % anim.get_frame_count()
    ret += 'Frame Time: %f\n' % anim.get_frame_length()
    for frame in range(0, anim.get_frame_count()):
	ret += _get_bvh_motion(anim.Character, root, 0, frame, anim.SolutionValues)
	ret += '\n'
    ret += '\n'

    return ret

def export_bvh_flat(anim):
    '''Returns a string containing the animation in BVH format with a
    "flat" hierarchy (every bone is parented to a bone at the origin). This
    was used for debugging and is probably not what you want.'''

    ret = ''

    #write hierarchy
    ret += 'HIERARCHY\n'
    ret += 'ROOT origin\n'
    ret += '{\n'
    ret += '\tOFFSET\t%f\t%f\t%f\n' % (0, 0, 0)
    ret += '\tCHANNELS 6 Xposition Yposition Zposition Xrotation Yrotation Zrotation\n'
    for body in anim.Character.BodyList:
	ret += '\tJOINT %s\n' % body.Name
	ret += '\t{\n'
	ret += '\t\tOFFSET\t%f\t%f\t%f\n' % (0, body.Diameter[1], 0)
	ret += '\t\tCHANNELS 6 Xposition Yposition Zposition Xrotation Yrotation Zrotation\n'
	ret += '\t\tEnd Site\n'
	ret += '\t\t{\n'
	ret += '\t\t\tOFFSET\t%f\t%f\t%f\n' % (0, -body.Diameter[1], 0)
	ret += '\t\t}\n'
	ret += '\t}\n'
    ret += '}\n'

    #write motion
    ret += 'MOTION\n'
    ret += 'Frames: %i\n' % anim.get_frame_count()
    ret += 'Frame Time: %f\n' % anim.get_frame_length()
    for frame in range(0, anim.get_frame_count()):
	ret += '%f %f %f %f %f %f ' % (0, 0, 0, 0, 0, 0) #root doesn't move
	for body in anim.Character.BodyList:
	    q = [anim.SolutionValues[str(body.q[x])][frame] for x in range(dof)]
	    qtx, qty, qtz = world_xf(body.ep_a(), q)
	    qrx = anim.SolutionValues[str(body.q[3])][frame] * (180.0 / math.pi)
	    qry = anim.SolutionValues[str(body.q[4])][frame] * (180.0 / math.pi)
	    qrz = anim.SolutionValues[str(body.q[5])][frame] * (180.0 / math.pi)
	    ret += ''.join(('%f ' % x) for x in [qtx, qty, qtz])
	    ret += ''.join(('%f ' % x) for x in [qrx, qry, qrz])
	ret += '\n'
    ret += '\n'

    return ret