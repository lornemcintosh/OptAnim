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
	bone.set("id", str(body.Id))
	bone.set("name", str(body.Name))

	position = ET.SubElement(bone, "position")
	if i == 0:
	    position.set("x", "%.9f" % rootoffset[0])
	    position.set("y", "%.9f" % rootoffset[1])
	    position.set("z", "%.9f" % rootoffset[2])
	else:
	    offset = [ body.ParentJoint.PointA[i] - body.Parent.ep_a()[i] for i in range(len(body.Parent.ep_a()))]
	    position.set("x", "%.9f" % offset[0])
	    position.set("y", "%.9f" % offset[1])
	    position.set("z", "%.9f" % offset[2])

	rotation = ET.SubElement(bone, "rotation")
	rotation.set("angle", "%.9f" % 0.0)
	axis = ET.SubElement(rotation, "axis")
	axis.set("x", "%.9f" % 1.0)
	axis.set("y", "%.9f" % 0.0)
	axis.set("z", "%.9f" % 0.0)

    bonehierarchy = ET.SubElement(root, "bonehierarchy")
    for i, joint in enumerate(anim.Character.JointList):
	if isinstance(joint, JointRevolute):
	    boneparent = ET.SubElement(bonehierarchy, "boneparent")
	    boneparent.set("bone", str(joint.BodyB.Name))
	    boneparent.set("parent", str(joint.BodyA.Name))

    animations = ET.SubElement(root, "animations")
    animation = ET.SubElement(animations, "animation")
    animation.set("name", str(anim.Name))
    length = (len(anim.AnimationData.items()[0][1])-1) * anim.get_frame_length()
    animation.set("length", str(length))
    tracks = ET.SubElement(animation, "tracks")

    for i, body in enumerate(anim.Character.BodyList):
	track = ET.SubElement(tracks, "track")
	track.set("bone", str(body.Name))
	keyframes = ET.SubElement(track, "keyframes")
	#for frame in range(anim.get_frame_count()):
	for frame in range(len(anim.AnimationData.items()[0][1])):
	    keyframe = ET.SubElement(keyframes, "keyframe")
	    keyframe.set("time", str(frame * anim.get_frame_length()))

	    bonepos = []
	    if i == 0:
		#get position of root
		q = anim.AnimationData[str(body.Name)][frame]
		bonepos = sym_world_xf(body.ep_a(), q)
		#in ogre, the root translation seems to be expressed relative to
		#its "root pose" specified above, so here we subtract it out
		bonepos[0] = bonepos[0] - rootoffset[0]
		bonepos[1] = bonepos[1] - rootoffset[1]
		bonepos[2] = bonepos[2] - rootoffset[2]
	    else:
		bonepos = [0.0]*3
	    translate = ET.SubElement(keyframe, "translate")
	    translate.set("x", "%.9f" % bonepos[0])
	    translate.set("y", "%.9f" % bonepos[1])
	    translate.set("z", "%.9f" % bonepos[2])

	    axisangle = [0.0]*4
	    if i == 0:
		rootEuler = anim.AnimationData[str(body.Name)][frame][3:dof]
		#convert to axis angle... by way of a quat :)
		quat = num_euler_to_quat(rootEuler)
                tmp = quat.toAngleAxis()
		axisangle = [tmp[1].x, tmp[1].y, tmp[1].z, tmp[0]]
	    else:
		childEuler = anim.AnimationData[str(body.Name)][frame][3:dof]
		childQuat = num_euler_to_quat(childEuler)

		parentEuler = anim.AnimationData[str(body.Parent.Name)][frame][3:dof]
		parentQuat = num_euler_to_quat(parentEuler)

		#express child relative to parent
                relativeQuat = parentQuat.inverse() * childQuat

		#convert to axis angle
                tmp = relativeQuat.toAngleAxis()
		axisangle = [tmp[1].x, tmp[1].y, tmp[1].z, tmp[0]]

	    rotate = ET.SubElement(keyframe, "rotate")
	    rotate.set("angle", "%.9f" % axisangle[3])
	    axis = ET.SubElement(rotate, "axis")
	    axis.set("x", "%.9f" % axisangle[0])
	    axis.set("y", "%.9f" % axisangle[1])
	    axis.set("z", "%.9f" % axisangle[2])

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
	ret += '%sCHANNELS 6 Xposition Yposition Zposition Yrotation Xrotation Zrotation\n' % tab
    else:
	#regular case
	ret += '%sJOINT %s\n' % (tab, root.Name)
	ret += '%s{\n' % tab
	level += 1; tab = '\t' * level;
	offset = [ root.ParentJoint.PointA[i] - root.Parent.ep_a()[i] for i in range(len(root.Parent.ep_a()))]
	ret += '%sOFFSET\t%f\t%f\t%f\n' % tuple([tab] + offset)
	ret += '%sCHANNELS 3 Yrotation Xrotation Zrotation\n' % tab
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
	q = data[str(body.Name)][frame]
	pos = sym_world_xf(body.ep_a(), q)
	pos = map(float, pos)	#evaluate sympy expression to a float
	ret += '{: .9f} {: .9f} {: .9f} '.format(*pos)

	rot = [float(q[x] * (180.0 / math.pi)) for x in range(3,dof)] #convert to degrees
	rot[0], rot[1] = rot[1], rot[0]	#swap x and y (so we output YXZ)
	ret += '{: .9f} {: .9f} {: .9f} '.format(*rot)

    #regular case
    else:
	#this will convert the rotations of the child and parent to matricies, express
	#the child relative to the parent, and convert them back to euler angles again
	'''
        childEuler = data[str(body.Name)][frame][3:dof]
	childMat = sym_euler_to_matrix(childEuler).evalf()

	parentEuler = data[str(body.Parent.Name)][frame][3:dof]
	parentMat = sym_euler_to_matrix(parentEuler).evalf()

	m = (parentMat.inv() * childMat).evalf() #express child relative to parent
	r = sym_matrix_to_euler(m)
        '''
        
        childEuler = data[str(body.Name)][frame][3:dof]
        childQuat = num_euler_to_quat(childEuler)

        parentEuler = data[str(body.Parent.Name)][frame][3:dof]
        parentQuat = num_euler_to_quat(parentEuler)

        #express child relative to parent
        relativeQuat = parentQuat.inverse() * childQuat

        #convert to euler
        r = num_quat_to_euler(relativeQuat)

	r = [float(x * (180.0 / math.pi)) for x in r]	#convert to degrees
	r[0], r[1] = r[1], r[0]	#swap x and y (so we output YXZ)
	ret += '{: .9f} {: .9f} {: .9f} '.format(*r)

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
    ret += 'Frames: %i\n' % len(anim.AnimationData.items()[0][1])
    ret += 'Frame Time: %f\n' % anim.get_frame_length()
    #for frame in range(anim.get_frame_count()):
    for frame in range(len(anim.AnimationData.items()[0][1])):
	ret += _get_bvh_motion(anim.Character, root, 0, frame, anim.AnimationData)
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
    ret += '\tCHANNELS 6 Xposition Yposition Zposition Yrotation Xrotation Zrotation\n'
    for body in anim.Character.BodyList:
	ret += '\tJOINT %s\n' % body.Name
	ret += '\t{\n'
	ret += '\t\tOFFSET\t%f\t%f\t%f\n' % (0, body.Diameter[1], 0)
	ret += '\t\tCHANNELS 6 Xposition Yposition Zposition Yrotation Xrotation Zrotation\n'
	ret += '\t\tEnd Site\n'
	ret += '\t\t{\n'
	ret += '\t\t\tOFFSET\t%f\t%f\t%f\n' % (0, -body.Diameter[1], 0)
	ret += '\t\t}\n'
	ret += '\t}\n'
    ret += '}\n'

    #write motion
    ret += 'MOTION\n'
    ret += 'Frames: %i\n' % len(anim.AnimationData.items()[0][1])
    ret += 'Frame Time: %f\n' % anim.get_frame_length()
    #for frame in range(0, anim.get_frame_count()):
    for frame in range(len(anim.AnimationData.items()[0][1])):
	ret += '%f %f %f %f %f %f ' % (0, 0, 0, 0, 0, 0) #root doesn't move
	for body in anim.Character.BodyList:
	    q = anim.AnimationData[str(body.Name)][frame]
	    qtx, qty, qtz = sym_world_xf(body.ep_a(), q)
	    qrx, qry, qrz = [x * (180.0 / math.pi) for x in anim.AnimationData[str(body.Name)][frame][3:dof]]
	    ret += ''.join(('%f ' % x) for x in [qtx, qty, qtz])
	    ret += ''.join(('%f ' % x) for x in [qry, qrx, qrz])    #YXZ
	ret += '\n'
    ret += '\n'

    return ret