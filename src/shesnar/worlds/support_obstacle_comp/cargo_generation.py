import os
import yaml

color_dict = {
    0: [0.0, 0.357, 0.549],    # [0, 91, 140] -> [0.0, 91/255, 140/255]
    1: [0.976, 0.498, 0.063],  # [249, 127, 16] -> [249/255, 127/255, 16/255]
    2: [0.165, 0.161, 0.165]   # [42, 41, 42] -> [42/255, 41/255, 42/255]
}
combinations_dict={
    0: (0,0,0,0),
    1: (0,0,0,1),
    2: (0,0,1,0),
    3: (0,0,1,1),
    4: (0,1,0,0),
    5: (0,1,0,1),
    6: (0,1,1,0),
    7: (0,1,1,1),
    8: (1,0,0,0),
    9: (1,0,0,1),
    10: (1,0,1,0),
    11: (1,0,1,1),
    12: (1,1,0,0),
    13: (1,1,0,1),
    14: (1,1,1,0),
    15: (1,1,1,1),

    #16: {2,2}# для черных грузов

}



def create_cargo_positions(input_path, output_path):
    ''' Code for generation sdf file with cargoes'''
    long, wide, height = 0.15, 0.05, 0.03
    with open(input_path, 'r') as f:
        config = yaml.safe_load(f)

    
    sdf_content = '''<?xml version="1.0"?>
<sdf version="1.9">
  <model name="all_cargo">
'''
    
    group=0
    for j, group_data in config.items():
        
        if not group_data.get('active', True): continue

        color_combination=combinations_dict.get(group_data['is_color'])
        
        if len(group_data['coords']) >= 6:
            group+=1
            x, y, z, x_rot, y_rot, z_rot = group_data['coords']
            
            if z_rot != 0: 
                x -= 0.075
            else: 
                y -= 0.075
                

            sdf_content += f'''    <model name='group_{group}'>\n'''

            for i in range(group_data['elements']):
                color=color_dict.get(color_combination[i])
                red, green, blue= color
                sdf_content += f'''
      <model name='cargo_{group}_{i+1}'>
        <pose>{x:.3f} {y:.3f} {z:.3f} {x_rot} {y_rot} {z_rot}</pose>
        <link name='grip_link_{group}_{i+1}'>
          <inertial>
            <inertia>
              <ixx>2.98e-4</ixx>
              <ixy>0</ixy>
              <ixz>0</ixz>
              <iyy>4.67e-3</iyy>
              <iyz>0</iyz>
              <izz>4.95e-3</izz>
            </inertia>
            <mass>0.35</mass>
            <pose>0 0 0 0 0 0</pose>
          </inertial>
          <collision name='box_collision_{group}_{i+1}'>
            <geometry>
              <box>
                <size>{long} {wide} {height}</size>
              </box>
            </geometry>
            <surface>
              <friction>
                <ode/>
              </friction>
              <bounce/>
              <contact/>
            </surface>
          </collision>
          <visual name='box_visual_{group}_{i+1}'>
            <geometry>
              <box>
                <size>{long} {wide} {height}</size>
              </box>
            </geometry>
            <material>
              <ambient> {red} {green} {blue} 1</ambient>
              <diffuse> {red} {green} {blue} 1</diffuse>
              <specular>1 1 1 1</specular>
            </material>
          </visual>
          <pose>0 0 0 0 0 0</pose>
          <enable_wind>false</enable_wind>
        </link>
        <static>false</static>
      </model>'''
                print(x,y,z, group)
                if z_rot != 0: 
                    x += 0.05
                else: 
                    y += 0.05

            sdf_content += f'\n    </model>'
    
    sdf_content += '\n  </model>\n</sdf>'
    
    with open(output_path, 'w') as f:
        f.write(sdf_content)
    
    print(f'Successfully created {group} cargo groups in {output_path}')

if __name__ == '__main__':
    base_dir = os.path.dirname(os.path.abspath(__file__))
    positions_path = os.path.join(base_dir, 'positions.yaml')
    output_path = os.path.join(base_dir, 'cargoes.sdf')
    
    create_cargo_positions(positions_path, output_path)