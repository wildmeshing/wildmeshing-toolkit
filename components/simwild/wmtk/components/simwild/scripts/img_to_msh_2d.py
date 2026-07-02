import cv2
from matplotlib.pyplot import contour
import numpy as np
import sys
import argparse


def write_msh(fname, pts, simplices, tags):
    """Write mesh to Gmsh MSH format."""
    with open(fname, "w") as f:
        # format
        f.write("$MeshFormat\n4.1 0 8\n$EndMeshFormat\n")

        # entities
        f.write("$Entities\n0 0 1 0\n")
        xmin, ymin = pts.min(axis=0)
        xmax, ymax = pts.max(axis=0)
        f.write(f"1 {xmin} {ymin} 0 {xmax} {ymax} 0 0 0\n")
        f.write("$EndEntities\n")

        # nodes
        f.write("$Nodes\n")
        num_nodes = pts.shape[0]
        f.write(f"1 {num_nodes} 1 {num_nodes}\n")
        f.write(f"2 1 0 {num_nodes}\n")
        for i in range(1, num_nodes + 1):
            f.write(f"{i}\n")
        for x, y in pts:
            f.write(f"{x} {y} 0\n")
        f.write("$EndNodes\n")

        # elements
        f.write("$Elements\n")
        num_elems = simplices.shape[0]
        f.write(f"1 {num_elems} 1 {num_elems} \n")
        f.write(f"2 1 2 {num_elems}\n")
        for eid, tri_nodes in enumerate(simplices, start=1):
            n1, n2, n3 = (tri_nodes + 1).tolist()
            f.write(f"{eid} {n1} {n2} {n3}\n")
        f.write("$EndElements\n")

        # labels
        f.write("$ElementData\n")
        f.write("1\n\"tag_0\"\n1\n0.0\n3\n0\n1\n")
        f.write(f"{num_elems}\n")
        for eid, tag in enumerate(tags, start=1):
            f.write(f"{eid} {int(tag)}\n")
        f.write("$EndElementData\n")


def subdivide_and_smooth(vertices, iterations=0, smoothing_steps=20, h=0.5):
    """
    Subdivides and smooths the boundary using Laplacian smoothing.

    Parameters:
        vertices (list of tuple): List of (x, y) vertices.
        iterations (int): Number of subdivision steps.
        smoothing_steps (int): Number of Laplacian smoothing steps.

    Returns:
        list of tuple: Smoothed vertices.
    """
    vertices = np.array(vertices, dtype=np.float64)

    for iteration in range(iterations):
        # Subdivide: Insert midpoints between consecutive vertices
        new_vertices = []
        for i in range(len(vertices)):
            p1 = vertices[i]
            p2 = vertices[(i + 1) % len(vertices)]
            midpoint = (p1 + p2) / 2
            new_vertices.append(p1)
            new_vertices.append(midpoint)
        vertices = np.array(new_vertices)

        # Visualize after subdivision
        subdiv_image = np.zeros((500, 500, 3), dtype=np.uint8)
        for i in range(len(vertices)):
            x1, y1 = vertices[i]
            x2, y2 = vertices[(i + 1) % len(vertices)]
            cv2.line(subdiv_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 1)
        cv2.imshow(f"Subdivision Iteration {iteration + 1}", subdiv_image)
        cv2.waitKey(0)

    for step in range(smoothing_steps):
        smoothed_vertices = np.copy(vertices)
        for i in range(len(vertices)):
            prev_idx = (i - 1) % len(vertices)
            next_idx = (i + 1) % len(vertices)
            smoothed_vertices[i] = (1-h)*vertices[i] + h*((vertices[prev_idx] + vertices[next_idx]) / 2)
        vertices = smoothed_vertices

        # Visualize after smoothing
        smooth_image = np.zeros((500, 500, 3), dtype=np.uint8)
        for i in range(len(vertices)):
            x1, y1 = vertices[i]
            x2, y2 = vertices[(i + 1) % len(vertices)]
            cv2.line(smooth_image, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 1)
        if step == 0 or step == smoothing_steps - 1:
            cv2.imshow(f"Smoothing Step {step + 1}", smooth_image)
            cv2.waitKey(0)

    return vertices.tolist()

def interactive_marker_segmentation(image):
    """
    Simple marker-based segmentation - one marker per region.
    Left click: mark current region
    Right click: switch to next region
    Press 'r': reset markers
    Press 's': apply segmentation
    Press 'q': quit
    """
    markers = np.zeros(image.shape[:2], dtype=np.int32)
    markers_display = image.copy()
    current_region = 1
    drawing = False
    
    # Predefined colors for different regions
    region_colors = [
        (0, 0, 0),      # Region 0 - black (background)
        (255, 0, 0),    # Region 1 - red
        (0, 255, 0),    # Region 2 - green
        (0, 0, 255),    # Region 3 - blue
        (255, 255, 0),  # Region 4 - yellow
        (255, 0, 255),  # Region 5 - magenta
        (0, 255, 255),  # Region 6 - cyan
        (128, 0, 0),    # Region 7
        (0, 128, 0),    # Region 8
        (0, 0, 128),    # Region 9
    ]
    
    def draw_marker(event, x, y, flags, param):
        nonlocal markers, markers_display, drawing, current_region
        
        if event == cv2.EVENT_LBUTTONDOWN:
            drawing = True
            cv2.circle(markers, (x, y), 1, current_region, -1)
            cv2.circle(markers_display, (x, y), 1, region_colors[current_region % len(region_colors)], -1)
        elif event == cv2.EVENT_MOUSEMOVE and drawing:
            cv2.circle(markers, (x, y), 1, current_region, -1)
            cv2.circle(markers_display, (x, y), 1, region_colors[current_region % len(region_colors)], -1)
        elif event == cv2.EVENT_LBUTTONUP:
            drawing = False
        elif event == cv2.EVENT_RBUTTONDOWN:
            current_region += 1
            print(f"Switched to region {current_region}")
    
    cv2.namedWindow("Mark Regions", cv2.WINDOW_NORMAL)
    cv2.setMouseCallback("Mark Regions", draw_marker)
    
    print("Instructions:")
    print("  Left click and drag: Mark current region")
    print("  Right click: Switch to next region")
    print("  'r': Reset markers")
    print("  's': Apply watershed segmentation")
    print("  'q': Quit")
    print(f"Current region: {current_region}")
    
    while True:
        cv2.imshow("Mark Regions", markers_display)
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('r'):  # Reset
            markers = np.zeros(image.shape[:2], dtype=np.int32)
            markers_display = image.copy()
            current_region = 1
            print("Reset markers")
        elif key == ord('s'):  # Segment
            break
        elif key == ord('q'):  # Quit
            cv2.destroyAllWindows()
            return None, None
    
    cv2.destroyWindow("Mark Regions")
    return markers, current_region

def load_segmented_image(image_path, interactive=False):
    """Load an already segmented image where pixel values represent region IDs."""
    # Load the image
    image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise FileNotFoundError(f"The file {image_path} could not be loaded.")
    
    # Treat pixel values as region IDs
    # Assuming: 0 = background, 1,2,3... = regions
    markers_result = image.astype(np.int32)
    
    # Show the loaded segmentation
    unique_values = np.unique(markers_result)
    print(f"Loaded segmented image with values: {unique_values}")
    
    if interactive:
        # Create colored visualization
        color_image = np.zeros((image.shape[0], image.shape[1], 3), dtype=np.uint8)
        region_colors = [
            (0, 0, 0), (255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0),
            (255, 0, 255), (0, 255, 255), (128, 0, 0), (0, 128, 0), (0, 0, 128)
        ]
        
        for val in unique_values:
            color_image[markers_result == val] = region_colors[val % len(region_colors)]
        
        cv2.imshow("Loaded Segmentation", color_image)
        cv2.waitKey(0)
    
    return markers_result

def segment_image(image_path):
    # Load the image
    image = cv2.imread(image_path)
    if image is None:
        raise FileNotFoundError(f"The file {image_path} could not be loaded.")
    
    # Interactive marker placement
    markers, num_regions = interactive_marker_segmentation(image)
    if markers is None:
        print("Segmentation cancelled")
        return
    
    print(f"Applying watershed with {num_regions} regions...")
    
    # Apply watershed segmentation
    markers_result = cv2.watershed(image, markers)
    
    # Smooth the segmentation using morphological operations
    # This smooths boundaries while preserving overall structure
    print("Smoothing segmentation boundaries...")
    
    # Process each region separately to maintain region IDs
    smoothed_markers = markers_result.copy()
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    
    for region_id in range(1, num_regions + 1):
        # Create mask for this region
        region_mask = (markers_result == region_id).astype(np.uint8)
        
        # Apply morphological closing (fills small holes and smooths boundaries)
        smoothed_mask = cv2.morphologyEx(region_mask, cv2.MORPH_CLOSE, kernel)
        
        # Apply morphological opening (removes small protrusions)
        smoothed_mask = cv2.morphologyEx(smoothed_mask, cv2.MORPH_OPEN, kernel)
        
        # Update the smoothed markers
        smoothed_markers[smoothed_mask == 1] = region_id
    
    markers_result = smoothed_markers
    
    # Replace -1 boundary pixels with nearest region using majority vote
    # This eliminates the boundary markers that watershed creates
    boundary_mask = (markers_result == -1)
    if np.any(boundary_mask):
        print(f"Assigning {np.sum(boundary_mask)} boundary pixels to regions...")
        # For each boundary pixel, assign it based on majority of neighbors
        boundary_coords = np.argwhere(boundary_mask)
        for y, x in boundary_coords:
            # Collect all neighboring region IDs (8-connected)
            neighbor_regions = []
            for dy in [-1, 0, 1]:
                for dx in [-1, 0, 1]:
                    if dy == 0 and dx == 0:
                        continue
                    ny, nx = y + dy, x + dx
                    if 0 <= ny < markers_result.shape[0] and 0 <= nx < markers_result.shape[1]:
                        neighbor_val = markers_result[ny, nx]
                        if neighbor_val >= 0:  # Valid region
                            neighbor_regions.append(neighbor_val)
            
            # Assign to most common neighbor region
            if neighbor_regions:
                from collections import Counter
                most_common = Counter(neighbor_regions).most_common(1)[0][0]
                markers_result[y, x] = most_common
    
    # Create colored segmented image
    segmented_color = np.zeros_like(image)
    region_colors = [
        (0, 0, 0), (255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0),
        (255, 0, 255), (0, 255, 255), (128, 0, 0), (0, 128, 0), (0, 0, 128)
    ]
    
    for label in range(num_regions + 1):
        segmented_color[markers_result == label] = region_colors[label % len(region_colors)]
    
    cv2.imshow("Watershed Segmentation Result", segmented_color)
    cv2.waitKey(0)

    return markers_result  # Return the actual watershed result, not the colored visualization

def mesh_segmented_image(markers_result, obj_path, use_triangle_quality=True, interactive=False):
    
    # markers_result contains region IDs: 0 for background, 1,2,3... for regions
    # No -1 boundaries should be present
    
    # Find all unique regions
    unique_regions = np.unique(markers_result)
    unique_regions = unique_regions[(unique_regions >= 0)]  # Include all non-negative region IDs
    
    print(f"Found {len(unique_regions)} unique regions: {unique_regions}")
    
    # Find and visualize contours for all regions
    all_contours = []
    contour_image = np.zeros_like(markers_result, dtype=np.uint8)
    contour_image = cv2.cvtColor(contour_image, cv2.COLOR_GRAY2BGR)
    
    contour_colors = [
        (255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0),
        (255, 0, 255), (0, 255, 255), (128, 0, 0), (0, 128, 0),
        (0, 0, 128), (128, 128, 0)
    ]
    
    for i, region_id in enumerate(unique_regions):
        # Create mask for this region directly from markers_result
        region_mask = (markers_result == region_id).astype(np.uint8) * 255
        
        # Find ALL contours for this region (including disconnected components)
        contours, _ = cv2.findContours(region_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
        all_contours.extend(contours)
        
        # Draw contours with different colors
        color = contour_colors[i % len(contour_colors)]
        cv2.drawContours(contour_image, contours, -1, color, 2)
        print(f"Region {region_id}: {len(contours)} contour(s), total area: {sum([cv2.contourArea(c) for c in contours]):.0f}")
    
    if interactive:
        cv2.imshow(f"All Contours ({len(all_contours)} total)", contour_image)
        cv2.waitKey(0)
    
    contours = all_contours

    # Process all contours without smoothing or subdividing
    if not contours:
        raise ValueError("No contours found in the image.")
    
    # Import triangle only when needed
    import triangle as tr
    
    # Collect all contour points and find nearby grid points
    height, width = markers_result.shape
    contour_mask = np.zeros((height, width), dtype=bool)
    
    all_contour_points = []
    for contour in contours:
        contour_points = [(int(point[0][0]), int(point[0][1])) for point in contour]
        for x, y in contour_points:
            if 0 <= x < width and 0 <= y < height:
                contour_mask[y, x] = True
        all_contour_points.extend(contour_points)
    
    # Choose vertices based on use_triangle_quality flag
    if use_triangle_quality:
        # Dilate contour mask by 1 pixel to include neighboring points
        from scipy.ndimage import binary_dilation
        nearby_mask = binary_dilation(contour_mask, structure=np.ones((3, 3)))
        
        # Get all nearby grid points
        nearby_y, nearby_x = np.where(nearby_mask)
        nearby_points = np.column_stack([nearby_x, nearby_y]).astype(np.float64)
        
        print(f"Using {len(nearby_points)} grid points near contours (within 1 pixel)")
    else:
        # Use all pixel grid points
        grid_x, grid_y = np.meshgrid(np.arange(width), np.arange(height))
        nearby_points = np.column_stack([grid_x.ravel(), grid_y.ravel()]).astype(np.float64)
        
        print(f"Using all {len(nearby_points)} pixel grid points")
    
    # Collect segments from all contours
    all_segments = []
    contour_point_to_idx = {}
    
    # Map contour points to indices in nearby_points
    for i, (x, y) in enumerate(nearby_points):
        contour_point_to_idx[(int(x), int(y))] = i
    
    for contour in contours:
        contour_points = [(int(point[0][0]), int(point[0][1])) for point in contour]
        
        if len(contour_points) < 3:  # Skip degenerate contours
            continue
        
        # Create segments using indices
        indices = [contour_point_to_idx[pt] for pt in contour_points if pt in contour_point_to_idx]
        
        if len(indices) >= 3:
            segments = [(indices[i], indices[(i + 1) % len(indices)]) for i in range(len(indices))]
            all_segments.extend(segments)
    
    segments_array = np.array(all_segments, dtype=np.int32)
    
    # Create triangle input dictionary
    tri_input = {
        'vertices': nearby_points,
        'segments': segments_array
    }
    
    print(f"Triangulating with {len(nearby_points)} vertices and {len(segments_array)} segments...")
    
    # Triangulate with boundary preservation
    if use_triangle_quality:
        # Use quality refinement for interiors
        # 'p' = planar straight line graph (preserve segments)
        # 'q30' = minimum angle 30 degrees for interior
        # 'a100' = maximum triangle area 100 for interior refinement
        tri_flags = 'pq30a100'
        print("Using Triangle with quality refinement")
    else:
        # Only preserve segments, no refinement (use only input vertices)
        tri_flags = 'p'
        print("Using only grid point vertices")
    
    tri_output = tr.triangulate(tri_input, tri_flags)
    
    # Extract mesh data
    mesh_vertices = tri_output['vertices']
    mesh_triangles = tri_output['triangles']
    orig_vertices = tri_output['vertices']
    
    print(f"Generated mesh with {len(mesh_vertices)} vertices and {len(mesh_triangles)} triangles")
    
    # Assign tags based on triangle region
    tags = np.zeros(len(mesh_triangles), dtype=np.int32)
    
    print("Assigning tags to triangles...")
    
    # Vectorized tag assignment using centroids
    # Compute all centroids at once
    centroids = orig_vertices[mesh_triangles].mean(axis=1)
    
    # Convert to pixel coordinates
    px = np.floor(centroids[:, 0]).astype(np.int32)
    py = np.floor(centroids[:, 1]).astype(np.int32)
    
    # Check bounds
    valid_mask = (py >= 0) & (py < markers_result.shape[0]) & (px >= 0) & (px < markers_result.shape[1])
    
    # Look up tags for valid centroids
    tags[valid_mask] = markers_result[py[valid_mask], px[valid_mask]]
    
    # Handle invalid tags (negative values) by checking neighbors
    invalid_mask = tags < 0
    if np.any(invalid_mask):
        print(f"Checking neighbors for {np.sum(invalid_mask)} triangles with invalid centroid tags...")
        invalid_indices = np.where(invalid_mask)[0]
        
        for i in invalid_indices:
            pxi, pyi = px[i], py[i]
            # Check 8-connected neighbors
            for dy in [-1, 0, 1]:
                for dx in [-1, 0, 1]:
                    ny, nx = pyi + dy, pxi + dx
                    if 0 <= ny < markers_result.shape[0] and 0 <= nx < markers_result.shape[1]:
                        neighbor_tag = markers_result[ny, nx]
                        if neighbor_tag >= 0:
                            tags[i] = neighbor_tag
                            break
                if tags[i] >= 0:
                    break
    
    # Check for any remaining invalid tags
    still_invalid = tags < 0
    if np.any(still_invalid):
        invalid_indices = np.where(still_invalid)[0]
        for i in invalid_indices:
            centroid = centroids[i]
            print(f"Triangle {i}: centroid ({centroid[0]:.2f}, {centroid[1]:.2f}) -> pixel ({px[i]}, {py[i]})")
            print(f"  Triangle vertices: {orig_vertices[mesh_triangles[i]]}")
            print(f"  Marker value at centroid: {markers_result[py[i], px[i]] if 0 <= py[i] < markers_result.shape[0] and 0 <= px[i] < markers_result.shape[1] else 'out of bounds'}")
    
    assert np.all(tags >= 0), f"Some triangles have invalid tag values: {np.sum(tags < 0)} triangles"
    
    
    print("Unique tags:", np.unique(tags))
    print("Tag counts:", {tag: np.sum(tags == tag) for tag in np.unique(tags)})
    
    # Write to MSH file
    write_msh(obj_path, mesh_vertices, mesh_triangles, tags)
    print(f"MSH file saved to {obj_path}")

# Example usage
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Mesh a segmented 2D image')
    parser.add_argument('image', help='Input 2D image (PNG)')
    parser.add_argument('-o', '--output', required=True, help='Output MSH mesh file')
    parser.add_argument('-s', '--segment', action='store_true',
                        help='Perform interactive segmentation (default: load as pre-segmented image)')
    parser.add_argument('-i', '--interactive', action='store_true',
                        help='Interactive mode - show windows and wait for user input')
    parser.add_argument('-t', '--triangle-quality', action='store_true', 
                        help='Use Triangle with quality refinement (default: use only grid points)')

    
    args = parser.parse_args()
    
    # When -s is used, -i is implied
    if args.segment:
        args.interactive = True
    
    if args.segment:
        # Perform interactive segmentation
        segmented_markers = segment_image(args.image)
    else:
        # Load pre-segmented image
        segmented_markers = load_segmented_image(args.image, interactive=args.interactive)
    
    mesh_segmented_image(segmented_markers, args.output, use_triangle_quality=args.triangle_quality, interactive=args.interactive)
