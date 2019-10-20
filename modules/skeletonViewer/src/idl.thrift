/******************************************************************************
 *                                                                            *
 * Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file idl.thrift
 * @authors: Ugo Pattacini <ugo.pattacini@iit.it>
 */

/**
 * skeletonViewer_IDL
 *
 * IDL Interface to skeletonViewer RPC services.
 */
service skeletonViewer_IDL
{
   /**
    * Set the camera position in world coordinates.
    * @param x is the x-coordinate (meters).
    * @param y is the y-coordinate (meters).
    * @param z is the z-coordinate (meters).
    * @return true/false on success/failure.
    */
   bool set_camera_position(1:double x, 2:double y, 3:double z);

   /**
    * Set the camera focal point in world coordinates.
    * @param x is the x-coordinate (meters).
    * @param y is the y-coordinate (meters).
    * @param z is the z-coordinate (meters).
    * @return true/false on success/failure.
    */
   bool set_camera_focalpoint(1:double x, 2:double y, 3:double z);

   /**
    * Set the camera view up direction in world coordinates.
    * @param x is the x-coordinate (meters).
    * @param y is the y-coordinate (meters).
    * @param z is the z-coordinate (meters).
    * @return true/false on success/failure.
    */
   bool set_camera_viewup(1:double x, 2:double y, 3:double z);

   /**
    * Create a line.
    * @param name is the name of the line.
    * @param x0 the x-coordinate of the line origin (meters).
    * @param y0 the y-coordinate of the line origin (meters).
    * @param z0 the z-coordinate of the line origin (meters).
    * @param x1 the x-coordinate of the line end (meters).
    * @param y1 the y-coordinate of the line end (meters).
    * @param z1 the z-coordinate of the line end (meters).
    * @param r is the red channel of the line color [0,1].
    * @param g is the green channel of the line color [0,1].
    * @param b is the blue channel of the line color [0,1].
    * @return true/false on success/failure.
    */
   bool create_line(1:string name,
                    2:double x0, 3:double y0, 4:double z0,
		    5:double x1, 6:double y1, 7:double z1,
		    8:double r, 9:double g, 10:double b);

   /**
    * Delete a line.
    * @param name is the name of the line to delete.
    * @return true/false on success/failure.
    */
   bool delete_line(1:string name);
}
