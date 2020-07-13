import os, sys

import bpy
import glob 

from math import cos, sin, pi
from mathutils import Vector

def look_at(cam, cam_point, azimuth, elevation, distance):
    a = azimuth*pi/180.
    e = elevation*pi/180.
    d = distance
    x = cos(a)*cos(e)*d
    y = sin(a)*cos(e)*d
    z = sin(e)*d
    cam_loc = Vector((x,y,z)) + cam_point

    print((x,y,z))
    print(cam_loc)

    direction = cam_point - cam_loc    
    rot_quat = direction.to_track_quat('-Z', 'Y')

    cam.location = cam_loc
    cam.rotation_euler = rot_quat.to_euler()

if __name__ == '__main__':
    
    argin = sys.argv[-1]    
    if argin==None:
        print('[Error] invalid path: ' + argin)
        sys.exit(1)

    files = []
    if os.path.isfile(argin):
        print(argin + ' is a file')
        files.append(argin)
    else:
        print("glob: " + argin)

        if argin.startswith('~'):
            argin = os.environ['HOME'] + argin[1:]

        for name in glob.glob(argin):
            print('add ' + name)
            files.append(name)

    if len(files)==0:
        print('[Error] invalid path: ' + argin)
        sys.exit(1)

    # delete default cube
    for ob in bpy.context.scene.objects:
        ob.select = ob.name.startswith("Cube")
    bpy.ops.object.delete()

    # add objects
    material = bpy.data.materials.new("obj")
    #material.diffuse_color = (150/255.,180/255.,255/255.)
    material.diffuse_color = (180/255.,200/255.,255/255.)
    
    #bpy.ops.mesh.primitive_plane_add(radius=3, location=(0,0,-0.26))
    bpy.ops.mesh.primitive_plane_add(radius=10, location=(0,0,-0.26))
    plane = bpy.context.selected_objects[0]
    plane.active_material = material
    
    mesh_list = []
    for file in files:
        print(file)
        bpy.ops.import_mesh.ply(filepath=file)
        mesh = bpy.context.selected_objects[0]
        mesh_list.append(mesh)
    
    center_sum = Vector((0,0,0))
    for mesh in mesh_list:
        mesh.active_material = material

        local_bbox_center = 0.125 * sum((Vector(b) for b in mesh.bound_box), Vector())
        global_bbox_center = mesh.matrix_world * local_bbox_center
        center_sum = global_bbox_center + center_sum
    center_sum = center_sum * 1.0 / len(mesh_list)

    # add Lamp
    #lamp_loc = (0.50, 0.0, 0.0)
    #lamp_loc = cam_loc[:]
    lamp_loc = (center_sum[0], center_sum[1], 0.30)
    bpy.ops.object.select_by_type(type='LAMP')
    bpy.ops.object.delete(use_global=False)
    bpy.context.scene.world.light_settings.use_environment_light = True
    bpy.context.scene.world.light_settings.environment_energy = 5
    bpy.context.scene.world.light_settings.environment_color = 'SKY_TEXTURE'
        
    bpy.ops.object.lamp_add(type='POINT', view_align = False, location=lamp_loc)
    lamp = bpy.data.objects['Point']
    lamp.data.use_specular = False
    lamp.data.shadow_method = 'RAY_SHADOW'
    lamp.data.shadow_ray_samples = 16
    lamp.data.shadow_soft_size = 0.8

    
    bpy.context.scene.render.resolution_x = 640*2
    bpy.context.scene.render.resolution_y = 480*2
    bpy.context.scene.render.resolution_percentage = 100

    # camera
    cam = bpy.context.scene.objects['Camera']
    #cam_point = Vector((0.50,0,-0.26))
    cam_point = center_sum

    bpy.context.scene.render.image_settings.color_depth = '16'
    bpy.context.scene.render.image_settings.file_format = 'JPEG'
    bpy.context.scene.render.image_settings.compression = 100

    azimuth_list = [180,45]
    for azimuth in azimuth_list:        
        look_at(cam, cam_point, azimuth, 30, 1)

        bpy.context.scene.render.filepath\
         = os.path.join(os.path.dirname(os.path.abspath(__file__)),'./%d.jpg' % azimuth)
    
        bpy.ops.render.render(write_still=True)

    #bpy.ops.file.autopack_toggle()
    bpy.ops.wm.save_as_mainfile(filepath='./tmp.blend')

    
    