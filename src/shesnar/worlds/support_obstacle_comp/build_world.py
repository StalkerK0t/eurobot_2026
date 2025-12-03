import os
import cargo_generation

#now unuseless

def build_world():
    base_dir = os.path.dirname(os.path.abspath(__file__))

    positions_path = os.path.join(base_dir, 'positions.yaml')
    cargo_output_path = os.path.join(base_dir, 'cargoes.sdf')
    world_template_path = os.path.join(base_dir, 'world_template.sdf')
    final_world_path = os.path.join('src/shesnar/worlds', 'complete_world.world')
    
    cargo_generation.create_cargo_positions(positions_path, cargo_output_path)
    
    with open(world_template_path, 'r') as f:
        world_content = f.read()
    
    with open(final_world_path, 'w') as f:
        f.write(world_content)
    
    return final_world_path

if __name__ == '__main__':
    build_world()