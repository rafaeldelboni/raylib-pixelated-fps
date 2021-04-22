/*******************************************************************************************
*   This example Copyright (c) 2018 Chris Camacho (codifies) http://bedroomcoders.co.uk/captcha/
********************************************************************************************/

#include "ode/collision.h"
#include "ode/mass.h"
#include "raylib.h"
#include "raymath.h"
#include "rlights.h"
#include <ode/ode.h>

// these are used by the collision callback, while they could be passed
// via the collision callbacks user data, making the global is easier
dWorldID world;
dJointGroupID contactgroup;

#define numObj 1  // 100 boxes, 100 spheres, 100 cylinders

// set a raylib model matrix from an ODE rotation matrix and position
void setTransform(const float pos[3], const float R[12], Matrix* matrix)
{
    matrix->m0 = R[0];
    matrix->m1 = R[4];
    matrix->m2 = R[8];
    matrix->m3 = 0;
    matrix->m4 = R[1];
    matrix->m5 = R[5];
    matrix->m6 = R[9];
    matrix->m7 = 0;
    matrix->m8 = R[2];
    matrix->m9 = R[6];
    matrix->m10 = R[10];
    matrix->m11 = 0;
    matrix->m12 = pos[0];
    matrix->m13 = pos[1];
    matrix->m14 = pos[2];
    matrix->m15 = 1;
}

// when objects potentially collide this callback is called
// you can rule out certain collisions or use different surface parameters
// depending what object types collide.... lots of flexibility and power here!
#define MAX_CONTACTS 8

static void nearCallback(void *data, dGeomID o1, dGeomID o2)
{
    (void)data;
    int i;
    // if (o1->body && o2->body) return;

    // exit without doing anything if the two bodies are connected by a joint
    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);
    if (b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact))
        return;

    dContact contact[MAX_CONTACTS]; // up to MAX_CONTACTS contacts per body-body
    for (i = 0; i < MAX_CONTACTS; i++) {
        contact[i].surface.mode = dContactBounce ;//| dContactSoftCFM;
        contact[i].surface.mu = dInfinity;
        contact[i].surface.mu2 = 0;
        contact[i].surface.bounce = 0.5;
        contact[i].surface.bounce_vel = 0.5;
        //contact[i].surface.soft_cfm = 0.01;
    }
    int numc = dCollide(o1, o2, MAX_CONTACTS, &contact[0].geom,
                        sizeof(dContact));
    if (numc) {
        dMatrix3 RI;
        dRSetIdentity(RI);
        for (i = 0; i < numc; i++) {
            dJointID c =
                dJointCreateContact(world, contactgroup, contact + i);
            dJointAttach(c, b1, b2);
        }
    }

}


int main(void)
{
    // Initialization
    //--------------------------------------------------------------------------------------
    const int screenWidth = 1920/2;
    const int screenHeight = 1080/2;

    // a space can have multiple "worlds" for example you might have different
    // sub levels that never interact, or the inside and outside of a building
    dSpaceID space;

    // create an array of bodies
    dBodyID obj[numObj];

    SetConfigFlags(FLAG_MSAA_4X_HINT);  // Enable Multi Sampling Anti Aliasing 4x (if available)
    InitWindow(screenWidth, screenHeight, "raylib [models] example - simple lighting material");

    // Define the camera to look into our 3d world
    Camera camera = {(Vector3){ 10.0f, 10.0f, 10.0f }, (Vector3){ 0.0f, 0.5f, 0.0f },
                        (Vector3){ 0.0f, 1.0f, 0.0f }, 45.0f, CAMERA_PERSPECTIVE};

    Model box = LoadModelFromMesh(GenMeshCube(1,1,1));
    Model ball = LoadModelFromMesh(GenMeshSphere(.5,32,32));
    Model cylinder = LoadModelFromMesh(GenMeshCylinder(.5,1,32));
    Model plane = LoadModel("resources/grass-plane.obj"); // Load the animated model mesh and basic data

    // texture the models
    Texture texture = LoadTexture("resources/test.png");
    Texture2D texturePlane = LoadTexture("resources/grass-texture.png");

    box.materials[0].maps[MAP_DIFFUSE].texture = texture;
    ball.materials[0].maps[MAP_DIFFUSE].texture = texture;
    cylinder.materials[0].maps[MAP_DIFFUSE].texture = texture;
    plane.materials[0].maps[MAP_DIFFUSE].texture = texturePlane;

    Shader shader = LoadShader("resources/simpleLight.vs", "resources/simpleLight.fs");
    // load a shader and set up some uniforms
    shader.locs[SHADER_LOC_MATRIX_MODEL] = GetShaderLocation(shader, "matModel");
    shader.locs[SHADER_LOC_VECTOR_VIEW] = GetShaderLocation(shader, "viewPos");

    // ambient light level
    int amb = GetShaderLocation(shader, "ambient");
    SetShaderValue(shader, amb, (float[4]){0.2,0.2,0.2,1.0}, SHADER_UNIFORM_VEC4);

    // models share the same shader
    box.materials[0].shader = shader;
    ball.materials[0].shader = shader;
    plane.materials[0].shader = shader;

    // using 4 point lights, white, red, green and blue
    Light lights[MAX_LIGHTS];
    lights[0] = CreateLight(LIGHT_POINT, (Vector3){ 25,25,25 }, Vector3Zero(),
                    WHITE, shader);
    lights[1] = CreateLight(LIGHT_POINT, (Vector3){ 25,25,-25 }, Vector3Zero(),
                    RED, shader);
    lights[2] = CreateLight(LIGHT_POINT, (Vector3){ -25,25,-25 }, Vector3Zero(),
                    GREEN, shader);
    lights[3] = CreateLight(LIGHT_POINT, (Vector3){ -25,25,25 }, Vector3Zero(),
                    BLUE, shader);

    SetCameraMode(camera, CAMERA_ORBITAL);     // Set camera mode

    SetTargetFPS(60);   // Set our game to run at 60 frames-per-second

    dInitODE2(0);   // initialise and create the physics
    world = dWorldCreate();
    space = dHashSpaceCreate(NULL);
    contactgroup = dJointGroupCreate(0);
    dWorldSetGravity(world, 0, -9.8, 0);    // gravity

    // create some decidedly sub optimal indices!
    int nV = plane.meshes[0].vertexCount;
    int *groundInd = RL_MALLOC(nV*sizeof(int));
    for (int i = 0; i<nV; i++ ) {
        groundInd[i] = i;
    }
    dTriMeshDataID triData = dGeomTriMeshDataCreate();
    dGeomTriMeshDataBuildSingle(triData, plane.meshes[0].vertices,
                            3 * sizeof(float), nV,
                            groundInd, nV,
                            3 * sizeof(int));
    dCreateTriMesh(space, triData, NULL, NULL, NULL);

    int indexCount = cylinder.meshes[0].vertexCount;
    int *objInd = RL_MALLOC(indexCount*sizeof(int));
    for (int i = 0; i<indexCount; i++ ) {
        objInd[i] = i;
    }
    dTriMeshDataID new_tmdata = dGeomTriMeshDataCreate();
    dGeomTriMeshDataBuildSingle(new_tmdata, cylinder.meshes[0].vertices, 3 * sizeof(float), indexCount, 
            objInd, indexCount, 3 * sizeof(dTriIndex));
    dGeomTriMeshDataPreprocess2(new_tmdata, (1U << dTRIDATAPREPROCESS_BUILD_FACE_ANGLES), NULL);

    // create the physics bodies
    for (int i = 0; i < numObj; i++) {
        obj[i] = dBodyCreate(world);
        dGeomID geom;
        dMatrix3 R;
        dMass m;
        // create either a box or sphere with the apropriate mass
        /*if (i<100) {*/
            /*geom = dCreateBox(space, 1,1,1);*/
            /*dMassSetBoxTotal(&m, 1, 0.5, 0.5, 0.5);*/
        /*} else if (i<200) {*/
            /*geom = dCreateSphere(space,0.5);*/
            /*dMassSetSphereTotal(&m, 1, 0.5);*/
        /*} else {*/
            /*geom = dCreateCylinder(space, 0.5, 1.0);*/
            /*dMassSetCylinderTotal(&m, 1.0, 2, 0.5, 1.0);*/
        /*}*/

        // testing only cylinders
        /*geom = dCreateCylinder(space, 0.5, 1.0);*/
        /*dMassSetCylinderTotal(&m, 1.0, 2, 0.5, 1.0);*/

        geom = dCreateTriMesh(space, new_tmdata, 0, 0, 0);
        dMassSetTrimesh( &m, 1.0, geom  );
        dGeomSetPosition(geom , -m.c[0], -m.c[1], -m.c[2]);
        dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);

        // give the body a random position and rotation
        dBodySetPosition(obj[i],
                dRandReal() * 10 - 5,
                4+(i/10),
                dRandReal() * 10 - 5);

        dRFromAxisAndAngle(R,
                dRandReal() * 2.0 - 1.0,
                dRandReal() * 2.0 - 1.0,
                dRandReal() * 2.0 - 1.0,
                dRandReal() * 10.0 - 5.0);

        dBodySetRotation(obj[i], R);
        // set the bodies mass and the newly created geometry
        dBodySetMass(obj[i], &m);
        dGeomSetBody(geom, obj[i]);
    }

    //--------------------------------------------------------------------------------------
    // Main game loop
    //--------------------------------------------------------------------------------------
    while (!WindowShouldClose())            // Detect window close button or ESC key
    {
        //--------------------------------------------------------------------------------------
        // Update
        //----------------------------------------------------------------------------------

        // enable or disable the lights depending on key press
        if (IsKeyPressed(KEY_L)) { lights[0].enabled = !lights[0].enabled; UpdateLightValues(shader, lights[0]);}
        if (IsKeyPressed(KEY_R)) { lights[1].enabled = !lights[1].enabled; UpdateLightValues(shader, lights[1]);}
        if (IsKeyPressed(KEY_G)) { lights[2].enabled = !lights[2].enabled; UpdateLightValues(shader, lights[2]);}
        if (IsKeyPressed(KEY_B)) { lights[3].enabled = !lights[3].enabled; UpdateLightValues(shader, lights[3]);}

        UpdateCamera(&camera);              // Update camera

        // check for collisions
        dSpaceCollide(space, 0, &nearCallback);
        // step the world
        dWorldQuickStep(world, 1. / 60.0);
        dJointGroupEmpty(contactgroup);

        //----------------------------------------------------------------------------------
        // Draw
        //----------------------------------------------------------------------------------
        BeginDrawing();

            ClearBackground(BLACK);

            BeginMode3D(camera);
                // draw markers to show where the lights are
                if (lights[0].enabled) { DrawCube(lights[0].position,.2,.2,.2,WHITE); }
                if (lights[1].enabled) { DrawCube(lights[1].position,.2,.2,.2,RED); }
                if (lights[2].enabled) { DrawCube(lights[2].position,.2,.2,.2,GREEN); }
                if (lights[3].enabled) { DrawCube(lights[3].position,.2,.2,.2,BLUE); }

                int spaceDown = IsKeyDown(KEY_SPACE);

                for (int i = 0; i < numObj; i++) {
                    // apply force if the space key is held
                    if (spaceDown) {
                        dBodyAddForce(obj[i],
                            dRandReal()*40-20, 20.0f, dRandReal()*40-20);
                        dBodyEnable (obj[i]); // case its gone to sleep
                    }

                    float* pos = (float *) dBodyGetPosition(obj[i]);
                    float* rot = (float *) dBodyGetRotation(obj[i]);

                    if(fabs(pos[0]) > 45 || fabs(pos[2]) > 45) {
                        // teleport back if too far away
                        dBodySetPosition(obj[i], dRandReal() * 10 - 5,
                                                8, dRandReal() * 10 - 5);
                        dBodySetLinearVel(obj[i], 0, 0, 0);
                        dBodySetAngularVel(obj[i], 0, 0, 0);
                        pos = (float *) dBodyGetPosition(obj[i]);
                    }

                    // set transform takes the bodies position and rotation
                    // matrix from ODE and inserts it into the models
                    // transform matrix
                    /*if (i<100) {*/
                        /*setTransform(pos, rot, &box.transform);*/
                        /*DrawModel(box, (Vector3){0,0,0}, 1.0f, WHITE);*/
                    /*} else if (i<200) {*/
                        /*setTransform(pos, rot, &ball.transform);*/
                        /*DrawModel(ball, (Vector3){0,0,0}, 1.0f, WHITE);*/
                    /*} else {*/
                        /*setTransform(pos, rot, &cylinder.transform);*/
                        /*DrawModel(cylinder, (Vector3){0,0,0}, 1.0f, WHITE);*/
                    /*}*/
                    setTransform(pos, rot, &cylinder.transform);
                    DrawModel(cylinder, (Vector3){0,0,0}, 1.0f, WHITE);
                }

                DrawModel(plane, (Vector3){0,0,0}, 1.0f, WHITE);

            EndMode3D();

            DrawFPS(10, 10);
            DrawText("Keys RGB & L toggle lights", 10, 30, 20, WHITE);
            DrawText("Space to add force", 10, 50, 20, WHITE);

        EndDrawing();
        //----------------------------------------------------------------------------------
    }

    //--------------------------------------------------------------------------------------
    // De-Initialization
    //--------------------------------------------------------------------------------------
    UnloadModel(box);
    UnloadModel(ball);
    UnloadModel(plane);
    UnloadTexture(texture);
    UnloadTexture(texturePlane);

    free(groundInd);
    dJointGroupEmpty(contactgroup);
    dJointGroupDestroy(contactgroup);
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();

    CloseWindow();              // Close window and OpenGL context
    //--------------------------------------------------------------------------------------

    return 0;
}
