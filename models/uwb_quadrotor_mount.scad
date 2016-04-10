$fn = 100;
screw_thickness = 2;
screw_height = 4;
mount_screw_d = 4;
mount_y_size = 39.25;
plate_x_size = 2;
plate_y_size = 70;
sensor_height = 10;

module screw_hole(x, y, z, screw_r=mount_screw_d / 2, wall_r=screw_thickness, height=screw_height)
{
    difference()
    {
        translate([x - (screw_r + wall_r), y - (screw_r + wall_r), z - height / 2])
        cube([2 * (screw_r + wall_r), 2 * (screw_r + wall_r), height], center=false);

        translate([x, y - 1.4 * screw_r, z - height], center=false)
        rotate([0, 0, 45])
        cube([2 * screw_r, 2 * screw_r, 2 * height], center=false);
    }
}

// left mount screw
screw_hole(0, -mount_y_size / 2, 0,
    height=screw_height,
    screw_r=mount_screw_d/2);

// right mount screw
screw_hole(0, mount_y_size / 2, 0,
    height=screw_height,
    screw_r=mount_screw_d/2);

difference()
{
    union()
    {
        // back plate
        translate([-mount_screw_d - screw_thickness, -plate_y_size / 2, -screw_height / 2])
        cube([plate_x_size, plate_y_size, screw_height], center=false);

        // front plate
        translate([mount_screw_d, -plate_y_size / 2, -screw_height / 2])
        cube([plate_x_size, plate_y_size, screw_height], center=false);

        // left top sensor grip
        translate([mount_screw_d, plate_y_size / 2 - mount_screw_d - 2 * screw_thickness, 0])
        cube([plate_x_size, mount_screw_d + 2 * screw_thickness, sensor_height], center=false);

        // left bottom sensor grip
        translate([-mount_screw_d - screw_thickness, plate_y_size / 2 - mount_screw_d - 2 * screw_thickness, 0])
        cube([plate_x_size, mount_screw_d + 2 * screw_thickness, sensor_height], center=false);

        // right top sensor grip
        translate([mount_screw_d, -plate_y_size / 2, 0])
        cube([plate_x_size, mount_screw_d + 2 * screw_thickness, sensor_height], center=false);

        // right bottom sensor grip
        translate([-mount_screw_d - screw_thickness, -plate_y_size / 2, 0])
        cube([plate_x_size, mount_screw_d + 2 * screw_thickness, sensor_height], center=false);

        // top left sensor screw hole
        translate([mount_screw_d + plate_x_size / 2, 0, sensor_height + mount_screw_d])
        rotate([0, 90, 0])
        screw_hole(0, plate_y_size / 2 - mount_screw_d, -5,
            height=12,
            screw_r=mount_screw_d/2);

        translate([mount_screw_d + plate_x_size / 2, 0, sensor_height + mount_screw_d])
        rotate([0, 90, 0])
        screw_hole(0, -plate_y_size / 2 + mount_screw_d, -5,
            height=12,
            screw_r=mount_screw_d/2);
        }

        translate([-1, -plate_y_size / 2 + mount_screw_d / 2 - screw_thickness - 2, sensor_height - 2])
        cube([2, 4 + mount_screw_d + 2 * screw_thickness, 4 + mount_screw_d + 2 * screw_thickness], center=false);

        translate([-1, plate_y_size / 2 - mount_screw_d / 2 - screw_thickness - 6, sensor_height - 2])
        cube([2, 4 + mount_screw_d + 2 * screw_thickness, 4 + mount_screw_d + 2 * screw_thickness], center=false);

}
