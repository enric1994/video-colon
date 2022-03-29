import argparse
import bpy
from mathutils import Euler, Matrix, Quaternion, Vector
import random
import blender_utils as utils
utils.mute()
import os
import numpy as np
from PIL import Image
from tqdm import tqdm

# python generate.py --name test --size 1

def gen(dataset_version, total_videos):
    
    for video_id in tqdm(range(0, total_videos)):

        # Remove default cube and camera
        bpy.ops.object.select_all(action='SELECT')
        bpy.ops.object.delete()        

        # Create a bezier circle and enter edit mode.
        bpy.ops.curve.primitive_bezier_circle_add(radius=1.0,
                                            location=(0.0, 0.0, 0.0),
                                            enter_editmode=True)

        # Subdivide the curve by a number of cuts, giving the
        # random vertex function more points to work with.
        bpy.ops.curve.subdivide(number_cuts=4)

        # Randomize the vertices of the bezier circle.
        # offset [-inf .. inf], uniform [0.0 .. 1.0],
        # normal [0.0 .. 1.0], RNG seed [0 .. 10000].
        bpy.ops.transform.vertex_random(offset=1.0, uniform=0.1, normal=0.0, seed=0)

        # Scale the curve while in edit mode.
        bpy.ops.transform.resize(value=(2.0, 2.0, 3.0))

        # Return to object mode.
        bpy.ops.object.mode_set(mode='OBJECT')


        #



        # Assumes curve has already been created.
        curve = bpy.context.active_object
        spline = curve.data.splines[0]
        bezier_points = spline.bezier_points
        is_cyclic = spline.use_cyclic_u

        # Create camera
        bpy.ops.object.camera_add()
        camera = bpy.context.active_object
        bpy.context.scene.camera = camera
        camera.rotation_mode = 'QUATERNION'

        # Create three keyframes for each bezier point.
        key_frame_count = 3.0 * len(bezier_points)
        scene = bpy.context.scene
        frame_start = scene.frame_start
        frame_end = scene.frame_end
        frame_len = frame_end - frame_start
        frame_skip = int(max(1.0, frame_len / key_frame_count))
        frame_range = range(frame_start, frame_end, frame_skip)
        frame_to_percent = 1.0 / frame_len

        for frame in frame_range:
            # Set to current frame.
            scene.frame_set(frame)

            # Calculate coordinates for frame by converting to a percent.
            frame_percent = frame * frame_to_percent
            coord_tangent = bezier_multi_seg(knots=bezier_points,
                                            step=frame_percent,
                                            closed_loop=is_cyclic)

            # Set location.
            coord = coord_tangent['coord']
            camera.location = coord
            camera.keyframe_insert(data_path='location')

            # Set rotation based on tangent.
            tangent = coord_tangent['tangent']
            tangent.normalize()
            rotation = tangent.to_track_quat('-Z', 'Y')
            camera.rotation_quaternion = rotation
            camera.keyframe_insert(data_path='rotation_quaternion')
            
            
            
        #

        # Store a shortcut to the curve object'sbpy.data.
        obj_data =bpy.data.objects['BezierCircle'].data

        # Which parts of the curve to extrude ['HALF', 'FRONT', 'BACK', 'FULL'].
        obj_data.fill_mode = 'FULL'

        # Breadth of extrusion.
        obj_data.extrude = 0.1

        # Depth of extrusion.
        obj_data.bevel_depth = 0.2

        # Smoothness of the segments on the curve.
        obj_data.resolution_u = 20
        obj_data.render_resolution_u = 20

#
        
        # num_polyps = random.randint(1,8)
        
        # # Make polyps
        # for i in range(0,num_polyps):
        #     bpy.ops.object.mode_set(mode='OBJECT')
        #     if i > 0:
        #         object_name = 'Sphere.' + str(i).zfill(3)
        #     else:
        #         object_name = 'Sphere'
        #     bpy.ops.mesh.primitive_uv_sphere_add(segments=128, ring_count=128, location=(0.0, 0.0, 0.0), rotation=(0.0, 0.0, 0.0))

        #     bpy.data.objects[object_name].scale[0] = random.uniform(0.1, 0.5)
        #     bpy.data.objects[object_name].scale[1] = random.uniform(0.1, 0.5)
        #     bpy.data.objects[object_name].scale[2] = random.uniform(0.1, 0.5)

        #     bpy.data.objects[object_name].location[0] = base_location_x + random.uniform(-0.2, 0.2)
        #     bpy.data.objects[object_name].location[1] = base_location_y + random.uniform(-0.2, 0.2)
        #     bpy.data.objects[object_name].location[2] = base_location_z + random.uniform(-0.2, 0.2)
        #     bpy.ops.object.mode_set(mode='EDIT')
        #     bpy.ops.transform.vertex_random(offset=deformation/2, uniform=0.0, normal=0.0, seed=0)
        #     bpy.ops.mesh.vertices_smooth()
        #     bpy.ops.mesh.vertices_smooth()
        #     bpy.ops.mesh.vertices_smooth()
        #     bpy.ops.mesh.vertices_smooth()
        #     # bpy.ops.mesh.vertices_smooth()
        #     # bpy.ops.mesh.vertices_smooth()
        #     bpy.ops.object.mode_set(mode='OBJECT')

        #     #Create material
        #     # mat = bpy.data.materials.new(name="Material." + str(i))
        #     # random_shade_1 = random.uniform(0.6,1.2)
        #     # random_shade_2 = random.uniform(0.6,1.2)
        #     # random_shade_3 = random.uniform(0.6,1.2)
        #     # mat.diffuse_color=[0.800000 * random_shade_1, 0.18 * random_shade_2, 0.13 * random_shade_3]

        #     # tex = bpy.data.textures.new("SomeName." + str(i), 'IMAGE')
        #     # img = bpy.data.images.load(filepath=plain_color('polyp'))

        #     # tex.image = img
        #     # # tex.texture_coords = 'WINDOW'

        #     # slot = mat.texture_slots.add()
        #     # slot.texture = tex
        #     # slot.texture_coords = 'GLOBAL'


        #     # Apply material
        #     bpy.data.objects[object_name].data.materials.append(mat)


        # Set lighting
        lighting_config = {'l2': {
                'light_type': 'SUN',
                'position': [0,0,10],
                'rotation': [
                    random.uniform(0,4),
                    random.uniform(0,4),
                    random.uniform(0,4)
                ],
                'energy': random.uniform(2,2),
                'shadow': False,
                'color': [
                    1,1,1
                ]
            },
            'l3': {
                'light_type': 'POINT',
                'position': [2.5,0,0],
                'rotation': [
                    random.uniform(0,4),
                    random.uniform(0,4),
                    random.uniform(0,4)
                ],
                'energy': random.uniform(0,1),
                'shadow': False,
                'color': [
                    1,1,1
                ]
            },
            'l4': {
                'light_type': 'POINT',
                'position': [3.5,0,0],
                'rotation': [
                    random.uniform(0,4),
                    random.uniform(0,4),
                    random.uniform(0,4)
                ],
                'energy': random.uniform(0,1),
                'shadow': False,
                'color': [
                    1,1,1
                ]
            },
            'negative_1': {
                'light_type': 'POINT',
                'position': [1.5,0,0],
                'rotation': [
                    0,0,0
                ],
                'negative_light': True,
                'energy': random.uniform(0,1),
                'shadow': False,
                'color': [
                    1,1,1
                ]
            },
            'negative_2': {
                'light_type': 'POINT',
                'position': [1.5,0.2,0],
                'rotation': [
                    0,0,0
                ],
                'negative_light': True,
                'energy': random.uniform(0,1),
                'shadow': False,
                'color': [
                    1,1,1
                ]
            },
             'negative_3': {
                'light_type': 'POINT',
                'position': [1.5,-0.2,0],
                'rotation': [
                    0,0,0
                ],
                'negative_light': True,
                'energy': random.uniform(0,1),
                'shadow': False,
                'color': [
                    1,1,1
                ]
            }



        }
        set_lighting(lighting_config)

        


        # # # Save 3D object
        # blend_file_path = bpy.data.filepath
        # directory = os.path.dirname(blend_file_path)
        # os.makedirs('/blender/datasets/{}/mesh'.format(dataset_version), exist_ok=True)
        # target_file = os.path.join('/blender/datasets/{}/mesh'.format(dataset_version), str(image_number).zfill(8) + '.obj')

        # bpy.ops.export_scene.obj(filepath=target_file)


        # # Render depth
        # os.makedirs('/blender/datasets/{}/depth'.format(dataset_version), exist_ok=True)
        # bpy.data.scenes['Scene'].use_nodes=True

        # bpy.context.scene.use_nodes = True
        # nodes = bpy.context.scene.node_tree.nodes
        # output_file = nodes.new("CompositorNodeOutputFile")


        # output_file.base_path = '/blender/datasets/{}/depth/'.format(dataset_version)
        # output_file.format.file_format = 'OPEN_EXR'
        # tree = bpy.context.scene.node_tree
        # links = tree.links

        # links.new(tree.nodes[2].inputs[0], tree.nodes[1].outputs[2])

        # bpy.ops.render.render(write_still=True)
        # source_file = os.path.join('/blender/datasets/{}/depth'.format(dataset_version), 'Image0001.exr')
        # target_file = os.path.join('/blender/datasets/{}/depth'.format(dataset_version), str(image_number).zfill(8) + '.exr')
        # os.rename(source_file, target_file)


        utils.save_project('/blender/scene.blend')

        # # Render
        # utils.render_keyframes('images', image_number, dataset_version)

        os.makedirs('/blender/datasets/{}/sequence_{}/frames'.format(dataset_version, str(video_id).zfill(8)), exist_ok=True)
        for i in range(1, 250):
            bpy.context.scene.frame_current = i
            bpy.context.scene.render.image_settings.file_format = 'PNG'
            bpy.context.scene.render.filepath = '/blender/datasets/{}/sequence_{}/frames/{}.png'.format(dataset_version, str(video_id).zfill(8), str(i).zfill(8)) 
            bpy.ops.render.render(write_still=True)

        # # Render mask
        # bpy.data.worlds['World'].horizon_color=(0,0,0)
        # for light_name in lighting_config.keys():
        #     bpy.data.objects[light_name].hide_render=True

        # for i in range(0,num_polyps):
        #     if i > 0:
        #         object_name = 'Sphere.' + str(i).zfill(3)
        #     else:
        #         object_name = 'Sphere'
        #     for slot in bpy.data.objects[object_name].material_slots:
        #         new_mat = bpy.data.materials.new(name="Mask")
        #         new_mat.diffuse_color = (1,1,1)
        #         slot.material = new_mat
        #         slot.material.use_shadeless=True
                
        # # Hide other objects
        # # bpy.data.objects['MyCurveObject'].hide_render=True

        # # Render
        # utils.render_keyframes('masks', image_number, dataset_version)

        # # Reset
        # bpy.ops.wm.read_factory_settings(use_empty=True)

        # utils.unmute(desc)


    # def remove_empty_masks(dataset_name):

    #     print('Removing images with no polyps...')

    #     clean_dir = '/blender/datasets/{}/masks/'.format(dataset_name)
    #     images_dir = '/blender/datasets/{}/images/'.format(dataset_name)
    #     depth_dir = '/blender/datasets/{}/depth/'.format(dataset_name)
    #     mesh_dir = '/blender/datasets/{}/mesh/'.format(dataset_name)

    #     os.remove(depth_dir + '/Image0000.exr')

    #     images = os.listdir(clean_dir)

    #     for image in images:
    #         img = Image.open(clean_dir + image)

    #         im=np.asarray(img)
    #         polyp_pixels = np.count_nonzero(im)

    #         if polyp_pixels < 10000:
    #             os.remove(clean_dir + image)
    #             os.remove(images_dir + image)

    #             os.remove(depth_dir + image.replace('.png','.exr'))
    #             os.remove(mesh_dir + image.replace('.png', '.obj'))
    #             os.remove(mesh_dir + image.replace('.png', '.mtl'))

    #             # print(image)
    #             # import pdb;pdb.set_trace()
            

def set_lighting(lighting_info):
    for light_name, light_data in lighting_info.items():
        bpy.ops.object.lamp_add(type=light_data['light_type'], location=light_data['position'], rotation=light_data['rotation'])
        lamp = bpy.context.active_object.data
        lamp.energy = light_data['energy']
        if 'color' in light_data:
            lamp.color = light_data['color']
        if 'negative_light' in light_data:
            lamp.use_negative = light_data['negative_light']
        if light_data['shadow']:
            lamp.shadow_method = 'RAY_SHADOW'
        if 'specular' in light_data:
            lamp.use_specular = light_data['specular']
        bpy.context.selected_objects[0].name = light_name


def bezier_tangent(pt0=Vector(), pt1=Vector(), pt2=Vector(), pt3=Vector(), step=0.5):
    # Return early if step is out of bounds [0, 1].
    if step <= 0.0:
        return pt1 - pt0
    if step >= 1.0:
        return pt3 - pt2

    # Find coefficients.
    u = 1.0 - step
    ut6 = u * step * 6.0
    tsq3 = step * step * 3.0
    usq3 = u * u * 3.0

    # Find tangent and return.
    return (pt1 - pt0) * usq3 + (pt2 - pt1) * ut6 + (pt3 - pt2) * tsq3 

def bezier_step(pt0=Vector(), pt1=Vector(), pt2=Vector(), pt3=Vector(), step=0.0):
    # Return early if step is out of bounds [0, 1].
    if step <= 0.0:
        return pt0.copy()
    if step >= 1.0:
        return pt3.copy()

    # Find coefficients.
    u = 1.0 - step
    tcb = step * step
    ucb = u * u
    tsq3u = tcb * 3.0 * u
    usq3t = ucb * 3.0 * step
    tcb *= step
    ucb *= u

    # Find point and return.
    return pt0 * ucb + pt1 * usq3t + pt2 * tsq3u + pt3 * tcb

def bezier_multi_seg(knots=[], step=0.0, closed_loop=False):
    knots_len = len(knots)
    if knots_len == 1:
        knot = knots[0]
        coord = knot.co.copy()
        return {'coord': coord, 'tangent': knot.handle_right - coord}

    if closed_loop:
        scaled_t = (step % 1.0) * knots_len
        index = int(scaled_t)
        a = knots[index]
        b = knots[(index + 1) % knots_len]
    else:
        if step <= 0.0:
            knot = knots[0]
            coord = knot.co.copy()
            return {'coord': coord, 'tangent': knot.handle_right - coord}
        if step >= 1.0:
            knot = knots[-1]
            coord = knot.co.copy()
            return {'coord': coord, 'tangent': coord - knot.handle_left}

        scaled_t = step * (knots_len - 1)
        index = int(scaled_t)
        a = knots[index]
        b = knots[index + 1]

    pt0 = a.co
    pt1 = a.handle_right
    pt2 = b.handle_left
    pt3 = b.co
    u = scaled_t - index

    coord = bezier_step(pt0=pt0, pt1=pt1, pt2=pt2, pt3=pt3, step=u)
    tangent = bezier_tangent(pt0=pt0, pt1=pt1, pt2=pt2, pt3=pt3, step=u)
    return {'coord': coord, 'tangent': tangent}

parser = argparse.ArgumentParser()
parser.add_argument('--name', type=str, default="test", help='Dataset name')
parser.add_argument('--size', type=int, default=20, help='Dataset size')
args = parser.parse_args()

gen(args.name, args.size)