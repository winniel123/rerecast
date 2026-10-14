//! A region that surrounds an obstacle has the obstacle cut out of it.

use glam::{UVec3, Vec3A};
use rerecast::{AreaType, ConfigBuilder, Contour, HeightfieldBuilder, TriMesh};

/// Adds the surface of an axis-aligned box to `mesh`, every face wound to point out.
fn add_cuboid(mesh: &mut TriMesh, center: Vec3A, half: Vec3A) {
    let corner = |index: usize| {
        let sign = Vec3A::new(
            if index & 1 == 0 { -1.0 } else { 1.0 },
            if index & 2 == 0 { -1.0 } else { 1.0 },
            if index & 4 == 0 { -1.0 } else { 1.0 },
        );
        center + sign * half
    };
    let faces: [[usize; 4]; 6] = [
        [0, 2, 6, 4],
        [1, 3, 7, 5],
        [0, 1, 5, 4],
        [2, 3, 7, 6],
        [0, 1, 3, 2],
        [4, 5, 7, 6],
    ];
    for face in faces {
        for fan in 1..3 {
            let (a, mut b, mut c) = (corner(face[0]), corner(face[fan]), corner(face[fan + 1]));
            if (b - a).cross(c - a).dot((a + b + c) / 3.0 - center) < 0.0 {
                core::mem::swap(&mut b, &mut c);
            }
            let base = mesh.vertices.len() as u32;
            mesh.vertices.extend([a, b, c]);
            mesh.indices.push(UVec3::new(base, base + 1, base + 2));
            mesh.area_types.push(AreaType::NOT_WALKABLE);
        }
    }
}

/// Whether `contour` turns clockwise across the xz-plane, as a hole's contour does.
fn wound_backwards(contour: &Contour) -> bool {
    let vertices = &contour.vertices;
    let n = vertices.len();
    let twice: i64 = (0..n)
        .map(|i| {
            let a = vertices[i].0.as_i64vec3();
            let b = vertices[(i + n - 1) % n].0.as_i64vec3();
            a.x * b.z - b.x * a.z
        })
        .sum();
    twice < 0
}

/// A 0.6 m box standing on a 60 x 60 m floor, where one region surrounds it. Its contours
/// are the region's outline and, wound backwards, a hole round the box. Recast's
/// `rcBuildContours` merges the hole into the outline; without that, `into_polygon_mesh`
/// found no ear in the hole and failed with `InvalidContour`.
#[test]
fn a_hole_in_a_region_is_merged_into_its_outline() {
    let mut mesh = TriMesh::default();
    add_cuboid(
        &mut mesh,
        Vec3A::new(0.0, -0.5, 0.0),
        Vec3A::new(30.0, 0.5, 30.0),
    );
    add_cuboid(
        &mut mesh,
        Vec3A::new(22.0, 0.45, 5.0),
        Vec3A::new(0.3, 0.45, 0.3),
    );
    let aabb = mesh.compute_aabb().unwrap();
    let config = ConfigBuilder {
        agent_radius: 0.4,
        agent_height: 1.8,
        walkable_climb: 0.4,
        walkable_slope_angle: 45_f32.to_radians(),
        cell_size_fraction: 2.0,
        cell_height_fraction: 4.0,
        aabb,
        ..ConfigBuilder::default()
    }
    .build();
    mesh.mark_walkable_triangles(config.walkable_slope_angle);
    let mut heightfield = HeightfieldBuilder {
        aabb,
        cell_size: config.cell_size,
        cell_height: config.cell_height,
    }
    .build()
    .unwrap();
    heightfield
        .rasterize_triangles(&mesh, config.walkable_climb)
        .unwrap();
    heightfield.filter_low_hanging_walkable_obstacles(config.walkable_climb);
    heightfield.filter_ledge_spans(config.walkable_height, config.walkable_climb);
    heightfield.filter_walkable_low_height_spans(config.walkable_height);
    let mut compact = heightfield
        .into_compact(config.walkable_height, config.walkable_climb)
        .unwrap();
    compact.erode_walkable_area(config.walkable_radius);
    compact.build_distance_field();
    compact
        .build_regions(
            config.border_size,
            config.min_region_area,
            config.merge_region_area,
        )
        .unwrap();

    let contours = compact.build_contours(
        config.max_simplification_error,
        config.max_edge_len,
        config.contour_flags,
    );

    // Every contour left is an outline, wound forwards; a merged hole is left empty.
    let backwards: Vec<usize> = contours
        .contours
        .iter()
        .enumerate()
        .filter(|(_, contour)| wound_backwards(contour))
        .map(|(index, _)| index)
        .collect();
    assert_eq!(backwards, Vec::<usize>::new(), "contours wound backwards");
    contours
        .into_polygon_mesh(config.max_vertices_per_polygon)
        .unwrap();
}
