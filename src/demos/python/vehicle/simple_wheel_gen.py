import math
import numpy as np
import pandas as pd



# Global constants
shell_thickness = 0.02
max_edge_size_at_feature_edges = 0.002
num_CPs = 4


def validate_thickness_constraints(rad, width, cp_deviation, g_height, g_width, particle_spacing):
    """
    Validate that all geometry components meet minimum thickness requirements for SPH simulation.
    
    Parameters:
    -----------
    rad : float
        Wheel radius
    width : float
        Wheel width
    cp_deviation : float
        Control point deviation
    g_height : float
        Grouser height
    g_width : float
        Grouser width
    particle_spacing : float
        Particle spacing for SPH simulation
        
    Returns:
    --------
    bool
        True if all constraints are met, False otherwise
    """
    min_thickness = 2 * particle_spacing
    
    # Check shell thickness
    if shell_thickness < min_thickness:
        print(f"ERROR: Shell thickness ({shell_thickness:.4f}m) is less than minimum required thickness ({min_thickness:.4f}m)")
        return False
    
    # Check grouser width
    if g_width < min_thickness:
        print(f"ERROR: Grouser width ({g_width:.4f}m) is less than minimum required thickness ({min_thickness:.4f}m)")
        return False
    
    # Check grouser height
    if g_height < min_thickness:
        print(f"ERROR: Grouser height ({g_height:.4f}m) is less than minimum required thickness ({min_thickness:.4f}m)")
        return False
    
    # Check if wheel width can accommodate particles
    if width < min_thickness:
        print(f"ERROR: Wheel width ({width:.4f}m) is less than minimum required thickness ({min_thickness:.4f}m)")
        return False
    
    return True

def generate_wheel_body_layers(rad, width, particle_spacing):
    """
    Generate wheel body particles using layer-based approach.
    
    Parameters:
    -----------
    rad : float
        Wheel radius
    width : float
        Wheel width
    particle_spacing : float
        Spacing between particles
        
    Returns:
    --------
    list
        List of particle positions for wheel body
    """
    particles = []
    
    # Calculate number of radial layers (from inner to outer radius)
    inner_radius = rad - shell_thickness
    outer_radius = rad
    num_radial_layers = int((outer_radius - inner_radius) / particle_spacing) + 1
    
    # Calculate number of axial layers (z-direction)
    num_axial_layers = int(width / particle_spacing) + 1
    
    # Calculate number of angular points per layer
    # Use the outer radius to determine angular spacing
    circumference = 2 * math.pi * outer_radius
    num_angular_points = int(circumference / particle_spacing)
    
    # print(f"Wheel body layers: {num_radial_layers} radial, {num_axial_layers} axial, {num_angular_points} angular")
    
    # Generate particles layer by layer
    for r_layer in range(num_radial_layers):
        r = inner_radius + r_layer * particle_spacing
        
        for z_layer in range(num_axial_layers):
            z = -width/2 + z_layer * particle_spacing
            
            for theta_idx in range(num_angular_points):
                theta = theta_idx * 2 * math.pi / num_angular_points
                
                x = r * np.cos(theta)
                y = r * np.sin(theta)
                particles.append([x, y, z])
    
    return particles

def generate_grouser_layers(grouser_params, g_angle, g_num, particle_spacing):
    """
    Generate straight grouser particles using layer-based approach.
    
    Parameters:
    -----------
    grouser_params : dict
        Grouser parameters
    g_angle : float
        Angle between grousers
    g_num : int
        Grouser number
    particle_spacing : float
        Spacing between particles
        
    Returns:
    --------
    list
        List of particle positions for this grouser
    """
    particles = []
    
    # Get grouser parameters
    wheel_radius = grouser_params['wheel_radius']
    grouser_height = grouser_params['grouser_height']
    grouser_width = grouser_params['grouser_width']
    wheel_width = grouser_params['wheel_width']
    # Optional fan control: angle in degrees controlling side face slope in (radial x, circumferential y)
    # theta = 90 -> vertical sides (original straight rectangular cross-section)
    fan_theta_deg = float(grouser_params.get('fan_theta_deg', 90.0))
    # Convert to slope in y per unit x using phi = 90 - theta
    # Positive slope expands with height; negative compresses
    tan_phi = math.tan(math.radians(90.0 - fan_theta_deg))
    
    # Calculate number of layers
    s = particle_spacing
    num_height_layers = int(grouser_height / s) + 1
    num_circumferential_layers = int(grouser_width / s) + 1

    # cot(theta) = 1 / tan(theta); use 0 when theta ~ 90° to reproduce straight case
    if abs(90.0 - fan_theta_deg) < 1e-8:
        cot_theta = 0.0
    else:
        cot_theta = 1.0 / math.tan(math.radians(fan_theta_deg))

    # print(f"Grouser {g_num} (fan) layers: {num_height_layers} height, variable axial by cot(theta)={cot_theta:.4f}, {num_circumferential_layers} circumferential")

    # Layer-by-layer fill over height h from wheel_radius to wheel_radius + grouser_height
    for k in range(num_height_layers):
        h_k = wheel_radius + k * s
        h_k = min(h_k, wheel_radius + grouser_height)

        # Symmetric expansion on both sides in X (mapped to z in global)
        delta_x = (h_k - wheel_radius) * cot_theta
        x_min = -wheel_width/2 - delta_x 
        x_max =  wheel_width/2 + delta_x

        if x_max <= x_min:
            continue

        num_x_layers = int((x_max - x_min) / s) + 1

        for xi in range(num_x_layers):
            z = x_min + xi * s

            for c_layer in range(num_circumferential_layers):
                y_grouser = -grouser_width/2 + c_layer * s

                # Local radial coordinate equals current height layer h_k
                x_local = h_k

                # Rotate to global XY around origin by grouser azimuth
                rot_angle = g_num * g_angle
                cos_rot = np.cos(rot_angle)
                sin_rot = np.sin(rot_angle)

                x = x_local * cos_rot - y_grouser * sin_rot
                y = x_local * sin_rot + y_grouser * cos_rot

                particles.append([x, y, z])
    
    return particles

def generate_grouser_semicircle_layers(grouser_params, g_angle, g_num, particle_spacing):
    """
    Generate semi-circular grouser particles using layer-based approach.
    Bottom part is rectangular (flush with wheel), top part is semi-circular.
   
    
    Parameters:
    -----------
    grouser_params : dict
        Grouser parameters
    g_angle : float
        Angle between grousers
    g_num : int
        Grouser number
    particle_spacing : float
        Spacing between particles
        
    Returns:
    --------
    list
        List of particle positions for this grouser
    """
    particles = []
    
    # Get grouser parameters
    wheel_radius = grouser_params['wheel_radius']
    grouser_height = grouser_params['grouser_height']
    grouser_width = grouser_params['grouser_width']
    wheel_width = grouser_params['wheel_width']
    
    # Calculate number of layers
    num_radial_layers = int(grouser_height / particle_spacing) + 1
    num_circumferential_layers = int(grouser_width / particle_spacing) + 1
    num_axial_layers = int(wheel_width / particle_spacing) + 1
    
    # Bottom rectangular, top semi-circular
    # Semi-circle radius is limited by wheel width and grouser height so that
    # the total protrusion equals grouser_height when possible.
    semi_circle_radius = min(wheel_width / 2.0, grouser_height)
    
    # Determine rectangular base height based on grouser height
    # If grouser_height >= semi_circle_radius:
    #   - Rectangular base = grouser_height - semi_circle_radius
    #   - Semi-circle top = semi_circle_radius
    # If grouser_height == semi_circle_radius:
    #   - No rectangular base (rect_height = 0)
    #   - Entire grouser is semi-circular
    rect_height = max(0, grouser_height - semi_circle_radius)
    semicircle_height = semi_circle_radius  # Always use full semi-circle radius
    
    num_rect_radial_layers = int(rect_height / particle_spacing) + 1 if rect_height > 0 else 0
    num_semicircle_radial_layers = int(semicircle_height / particle_spacing) + 1
    
    arc_length = math.pi * semi_circle_radius
    num_arc_points = int(arc_length / particle_spacing) + 1
    
    # print(f"Grouser {g_num} (convex semi-circle) - rect_height: {rect_height:.4f}, semicircle_height: {semicircle_height:.4f}")
    # print(f"  Layers: {num_rect_radial_layers} rect radial, {num_semicircle_radial_layers} semi radial, {num_arc_points} arc, {num_circumferential_layers} circumferential")
    
    # Part 1: Rectangular base (flush with wheel surface) - only if rect_height > 0
    if num_rect_radial_layers > 0:
        for r_layer in range(num_rect_radial_layers):
            r_offset = wheel_radius + r_layer * particle_spacing
            
            for z_layer in range(num_axial_layers):
                z = -wheel_width/2 + z_layer * particle_spacing
                
                for c_layer in range(num_circumferential_layers):
                    y_grouser = -grouser_width/2 + c_layer * particle_spacing
                    x_grouser = r_offset
                    
                    rot_angle = g_num * g_angle
                    cos_rot = np.cos(rot_angle)
                    sin_rot = np.sin(rot_angle)
                    
                    x = x_grouser * cos_rot - y_grouser * sin_rot
                    y = x_grouser * sin_rot + y_grouser * cos_rot
                    
                    particles.append([x, y, z])
    
    # Part 2: Semi-circular cap (FILLED)
    # The outer boundary of the cap is a semi-circle of set radius.
    # For each z along that arc, we fill inward toward the base at
    # (wheel_radius + rect_height) using the particle spacing.
    base_radius_for_semicircle = wheel_radius + rect_height

    # Sample uniformly along z (constant spacing) to avoid clustering
    num_z_layers = int((2 * semi_circle_radius) / particle_spacing) + 1
    for z_layer in range(num_z_layers):
        z = -semi_circle_radius + z_layer * particle_spacing

        # Compute outer arc offset at this z from circle equation x^2 + z^2 = R^2
        x_arc_offset = math.sqrt(max(0.0, semi_circle_radius**2 - z**2))
        x_outer = base_radius_for_semicircle + x_arc_offset
        x_inner = base_radius_for_semicircle

        # Number of radial fill layers from base to the arc at this z
        fill_thickness = max(0.0, x_outer - x_inner)
        num_fill_layers = int(fill_thickness / particle_spacing) + 1

        for r_fill in range(num_fill_layers):
            x_local = x_inner + r_fill * particle_spacing

            for c_layer in range(num_circumferential_layers):
                y_grouser = -grouser_width/2 + c_layer * particle_spacing

                # Rotate to global coordinate system
                rot_angle = g_num * g_angle
                cos_rot = np.cos(rot_angle)
                sin_rot = np.sin(rot_angle)

                x = x_local * cos_rot - y_grouser * sin_rot
                y = x_local * sin_rot + y_grouser * cos_rot

                particles.append([x, y, z])

    return particles



def GenSimpleWheelPointCloud(rad=0.25, width=0.2, cp_deviation=0., g_height=0.02, g_width=0.005, 
                           g_density=12, particle_spacing=0.01, grouser_type='straight', 
                           filename="wheel_particles.csv", fan_theta_deg=90.0):
    """
    Generate a point cloud for a 3D wheel with grousers for SPH simulation.
    Simplified version with circular wheel (no Bézier curves).
    
    Parameters:
    -----------
    rad : float
        Wheel radius (excluding grousers)
    width : float
        Wheel width (axial direction)
    cp_deviation : float
        Control point deviation (ignored in simplified version)
    g_height : float
        Height of the grousers
    g_width : float
        Width (thickness) of the grousers
    g_density : int
        Number of grousers per revolution
    particle_spacing : float
        Spacing between particles in meters
    grouser_type : str
        Type of grouser to generate. Options:
        - 'straight': Straight grousers (default)
        - 'semi_circle_concave': Semi-circular grouser curving toward trailing edge ")"
        - 'semi_circle_convex': Semi-circular grouser curving toward leading edge "("
        - 'v_shape': V-shaped grouser with apex pointing backward
        - 'v_shape_inverted': Inverted V-shaped (Λ) grouser with apex pointing forward
    filename : str
        Output CSV filename
        
    Returns:
    --------
    np.array
        Array of particle positions [N, 3]
    """
    
    # Validate thickness constraints
    if not validate_thickness_constraints(rad, width, cp_deviation, g_height, g_width, particle_spacing):
        raise ValueError("Cannot generate wheel: thickness constraints not met")
    
    # Validate grouser type
    valid_grouser_types = ['straight', 'semi_circle']
    if grouser_type not in valid_grouser_types:
        raise ValueError(f"Invalid grouser_type '{grouser_type}'. Must be one of {valid_grouser_types}")
    
    # print(f"Generating circular wheel with {grouser_type} grousers, particle spacing: {particle_spacing:.4f}m")
    
    # The angle between 2 adjacent grousers
    g_angle = 2 * math.pi / g_density
    
    # Generate wheel body using layer-based approach
    # print("Generating wheel body...")
    wheel_particles = generate_wheel_body_layers(rad, width, particle_spacing)
    
    # Generate grousers using layer-based approach
    # print(f"Generating {grouser_type} grousers...")
    grouser_params = {
        'wheel_radius': rad,
        'grouser_height': g_height,
        'grouser_width': g_width,
        'wheel_width': width,
        # For straight grousers, allow fan variant controlled by theta. Ignored by semi-circle.
        'fan_theta_deg': fan_theta_deg
    }
    
    all_grouser_particles = []
    for g_num in range(g_density):
        # Route to appropriate grouser generator based on type
        if grouser_type == 'straight':
            grouser_particles = generate_grouser_layers(grouser_params, g_angle, g_num, particle_spacing)
        elif grouser_type == 'semi_circle':
            grouser_particles = generate_grouser_semicircle_layers(grouser_params, g_angle, g_num, particle_spacing)
        
        all_grouser_particles.extend(grouser_particles)
    
    # Combine all particles
    particles = wheel_particles + all_grouser_particles
    
    particles = np.array(particles)
    # print(f"Generated {len(particles):,} particles")
    
    # Save to CSV
    # df = pd.DataFrame(particles, columns=['x', 'y', 'z'])
    # df.to_csv(filename, index=False)
    # print(f"Point cloud saved to {filename}")
    
    return particles



if __name__ == "__main__":
    
    # Example 1: Straight grousers (original)
    print("\n=== Generating wheel with STRAIGHT grousers ===")
    particles_straight = GenSimpleWheelPointCloud(
        rad=0.09,           # 25cm radius
        width=0.08,          # 20cm width
        cp_deviation=0,     # No curvature
        g_height=0.025,       # 10cm grouser height
        g_width=0.02,       # 1cm grouser thickness
        g_density=8,       # 12 grousers per revolution
        particle_spacing=0.005,  # 0.5cm particle spacing
        grouser_type='straight',
        filename="wheel_straight.csv"
    )
    
    
    # Example 2: Semi-circle convex grousers "("
    print("\n=== Generating wheel with SEMI-CIRCLE CONVEX grousers ===")
    particles_semi_convex = GenSimpleWheelPointCloud(
        rad=0.25,
        width=0.2,
        cp_deviation=0,
        g_height=0.05,
        g_width=0.01,
        g_density=12,
        particle_spacing=0.005,
        grouser_type='semi_circle',
        filename="wheel_semi.csv"
    )

    # Example 3: Fan-like straight grousers with theta control
    print("\n=== Generating wheel with FAN-STRAIGHT grousers (theta=60°) ===")
    particles_fan = GenSimpleWheelPointCloud(
        rad=0.09,
        width=0.08,
        cp_deviation=0,
        g_height=0.025,
        g_width=0.02,
        g_density=8,
        particle_spacing=0.005,
        grouser_type='straight',
        filename="wheel_fan_60.csv",
        fan_theta_deg=60.0
    )
    
    
    print("\n=== All point clouds generated successfully! ===")

