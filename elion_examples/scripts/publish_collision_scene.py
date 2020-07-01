#!/usr/bin/env python
"""
Load work objects from urdf file.
Assume the objects are defined with respect to a global link
called 'world', or with respect to links in the urdf file.
(The latter implies that the links are defined in the correct order.)
"""
import sys
import rospy
import rospkg
import rosparam
import moveit_commander
import urdfpy

from geometry_msgs.msg import Vector3, Quaternion, Pose, PoseStamped

# where to find the collision scenes in the elion_example package
PACKAGE_NAME = "elion_examples"
REL_URDF_PATH = "/urdf/collision_scenes/"

# possible root links of the given urdf file
ROOT_LINK_NAMES = ["world", "work", "base_link", "panda_link0"]


def numpy_to_pose(arr):
    """ Numpy 4x4 array to geometry_msg.Pose

    Code from: https://github.com/eric-wieser/ros_numpy
    TODO move this to some utility module if I have one.
    """
    from tf import transformations
    assert arr.shape == (4, 4)

    trans = transformations.translation_from_matrix(arr)
    quat = transformations.quaternion_from_matrix(arr)

    return Pose(position=Vector3(*trans), orientation=Quaternion(*quat))


def remove_all_objects(scene):
    for name in scene.get_known_object_names():
        scene.remove_world_object(name)


def list_work_objects(package_name, rel_file_path):
    """ Look for files in the folder specified by rel_file_path
    in the given package.
     """
    import glob
    rospack = rospkg.RosPack()

    filepath = rospack.get_path(package_name)
    filepath += rel_file_path
    print("Looking in directory:\n{}".format(filepath))
    print("\nFound the following files at {}:".format(filepath))
    for file in glob.glob(filepath + "*.urdf"):
        print("\t" + file.replace(filepath, "").replace(".urdf", ""))

def parse_urdf_file(package_name, file_name, rel_file_path, root_link_names):
    """ Convert urdf file (xml) to python dict.

    Using the urdfpy package for now.
    Using the xml package from the standard library could be
    easier to understand. We can change this in the future
    if it becomes a mess.
    """
    rospack = rospkg.RosPack()

    filepath = rospack.get_path(package_name)
    filepath += rel_file_path

    urdf = urdfpy.URDF.load(filepath + file_name)

    d = {"links": {}, "joints": {}}

    for link in urdf.links:
        if link.name in root_link_names:
            # root links have no geometry
            continue
        else:
            d["links"][link.name] = parse_link(link, filepath)

    for joint in urdf.joints:
        p = PoseStamped()
        p.header.frame_id = joint.parent
        p.pose = numpy_to_pose(joint.origin)

        d["joints"][joint.name] = {
            "pose": p,
            "parent": joint.parent,
            "child": joint.child
        }
    return d


def parse_link(link, mesh_path):
    """ Assume a link has only a single collision object.
        Assume this collision object is a box.
        Assume the link named "world" has no collision objects.

    link: a urdfpy.urdf.Link object
    mesh_path: absolute path of the folder where we have to fine the stl files
    """
    assert len(link.collisions) == 1
    assert link.name != "world"
    assert link.name != "work"
    collision = link.collisions[0]
    if collision.geometry.box is not None:
        data = {"type": "box", "size": link.collisions[0].geometry.box.size}
    elif collision.geometry.mesh is not None:
        data = {
            "type": "mesh",
            "filename": mesh_path + collision.geometry.mesh.filename,
            "scale": collision.geometry.mesh.scale
        }
    else:
        raise Exception("No mesh of box collision geometry found.")

    return data

def publish_parsed_urdf(parsed_urdf, scene):
    """ Publish link geometry for every joint's child. """
    for name, joint in parsed_urdf["joints"].items():
        # get the child link data
        link = parsed_urdf["links"][joint["child"]]

        # publish the child links collision geometry
        if link["type"] == "box":
            scene.add_box(
                joint["child"],
                joint["pose"],
                link["size"]
            )
        else:
            scene.add_mesh(
                joint["child"],
                joint["pose"],
                link["filename"],
                link["scale"]
            )

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("One argument required:")
        print("<work_name>:\tpublish urdf file <work_name>.urdf.")
        print("-l:\tlist available collision scenes.")
        print("-c:\tclear all collision objects.")
        exit()
    else:
        command = sys.argv[1]
        if command == "-l":
            list_work_objects(PACKAGE_NAME, REL_URDF_PATH)
            exit()
        elif command == "-c":
            rospy.init_node("publish_work")
            scene = moveit_commander.PlanningSceneInterface()
            rospy.sleep(1.0)  # wait for the above things to setup
            remove_all_objects(scene)
            exit()
        else:
            file_name = command + ".urdf"

    rospy.init_node("publish_work")


    scene = moveit_commander.PlanningSceneInterface()
    rospy.sleep(1.0)  # wait for the above things to setup

    remove_all_objects(scene)

    work = parse_urdf_file(PACKAGE_NAME, file_name, REL_URDF_PATH, ROOT_LINK_NAMES)

    publish_parsed_urdf(work, scene)

    print("Done!")