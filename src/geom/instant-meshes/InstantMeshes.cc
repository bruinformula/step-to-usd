/*
    main.cpp: CLI entry point for the bare-minimum Instant Meshes pipeline.

    Original work:
        Instant Field-Aligned Meshes
        Wenzel Jakob, Daniele Panozzo, Marco Tarini, and Olga Sorkine-Hornung
        In ACM Transactions on Graphics (Proc. SIGGRAPH Asia 2015)

    All rights reserved. Use of this source code is governed by a
    BSD-style license that can be found in the LICENSE.txt file.
*/

#include "mesher.h"
#include "common.h"
#include <iostream>
#include <cstring>
#include <cstdlib>

int nprocs = -1;

static void print_usage(const char *progname) {
    std::cout
        << "Syntax: " << progname << " [options] <input> -o <output>\n"
        << "\n"
        << "Required:\n"
        << "   -o, --output <file>       Output PLY or OBJ path\n"
        << "\n"
        << "Sizing (pick at most one):\n"
        << "   -s, --scale <length>      Target world-space edge length\n"
        << "   -f, --faces <count>       Target face count\n"
        << "   -v, --vertices <count>    Target vertex count\n"
        << "\n"
        << "Field options:\n"
        << "   -r, --rosy <2|4|6>        Orientation symmetry type (default: 4)\n"
        << "   -p, --posy <4|6>          Position symmetry type    (default: 4)\n"
        << "   -i, --intrinsic           Use intrinsic smoothness energy\n"
        << "   -b, --boundaries          Align output edges to mesh boundaries\n"
        << "   -D, --dominant            Dominant (mixed) mesh instead of pure tri/quad\n"
        << "\n"
        << "Pre-processing:\n"
        << "   -c, --crease <degrees>    Dihedral angle threshold for sharp creases\n"
        << "   -S, --smooth <iter>       Post-process smoothing iterations (default: 2)\n"
        << "   -k, --knn <count>         Point-cloud: neighbours to consider (default: 10)\n"
        << "\n"
        << "Misc:\n"
        << "   -t, --threads <count>     Worker thread count (default: automatic)\n"
        << "   -d, --deterministic       Prefer slower but deterministic algorithms\n"
        << "   -h, --help                Show this message\n";
}

int main(int argc, char **argv) {
    // Defaults
    std::string input_path, output_path;

    bool extrinsic          = true;
    bool align_to_boundaries = false;
    bool dominant           = false;
    bool deterministic      = false;
    bool help               = false;

    int   rosy         = 4;
    int   posy         = 4;
    int   face_count   = -1;
    int   vertex_count = -1;
    uint32_t knn_points   = 10;
    uint32_t smooth_iter  = 2;
    Float crease_angle = -1.f;
    Float scale        = -1.f;

    // Argument parsing
    try {
        for (int i = 1; i < argc; ++i) {
            auto next = [&](const char *flag) -> const char * {
                if (++i >= argc) {
                    std::cerr << "Missing argument for " << flag << "\n";
                    std::exit(-1);
                }
                return argv[i];
            };

            if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")) {
                help = true;
            } else if (!strcmp(argv[i], "-d") || !strcmp(argv[i], "--deterministic")) {
                deterministic = true;
            } else if (!strcmp(argv[i], "-i") || !strcmp(argv[i], "--intrinsic")) {
                extrinsic = false;
            } else if (!strcmp(argv[i], "-b") || !strcmp(argv[i], "--boundaries")) {
                align_to_boundaries = true;
            } else if (!strcmp(argv[i], "-D") || !strcmp(argv[i], "--dominant")) {
                dominant = true;
            } else if (!strcmp(argv[i], "-t") || !strcmp(argv[i], "--threads")) {
                nprocs = str_to_uint32_t(next("--threads"));
            } else if (!strcmp(argv[i], "-S") || !strcmp(argv[i], "--smooth")) {
                smooth_iter = str_to_uint32_t(next("--smooth"));
            } else if (!strcmp(argv[i], "-k") || !strcmp(argv[i], "--knn")) {
                knn_points = str_to_uint32_t(next("--knn"));
            } else if (!strcmp(argv[i], "-c") || !strcmp(argv[i], "--crease")) {
                crease_angle = str_to_float(next("--crease"));
            } else if (!strcmp(argv[i], "-r") || !strcmp(argv[i], "--rosy")) {
                rosy = str_to_int32_t(next("--rosy"));
            } else if (!strcmp(argv[i], "-p") || !strcmp(argv[i], "--posy")) {
                posy = str_to_int32_t(next("--posy"));
                if (posy == 6) posy = 3; // legacy alias
            } else if (!strcmp(argv[i], "-s") || !strcmp(argv[i], "--scale")) {
                scale = str_to_float(next("--scale"));
            } else if (!strcmp(argv[i], "-f") || !strcmp(argv[i], "--faces")) {
                face_count = str_to_int32_t(next("--faces"));
            } else if (!strcmp(argv[i], "-v") || !strcmp(argv[i], "--vertices")) {
                vertex_count = str_to_int32_t(next("--vertices"));
            } else if (!strcmp(argv[i], "-o") || !strcmp(argv[i], "--output")) {
                output_path = next("--output");
            } else if (argv[i][0] != '-') {
                if (!input_path.empty()) {
                    std::cerr << "Unexpected positional argument: " << argv[i] << "\n";
                    help = true;
                } else {
                    input_path = argv[i];
                }
            } else {
                std::cerr << "Unknown argument: " << argv[i] << "\n";
                help = true;
            }
        }
    } catch (const std::exception &e) {
        std::cerr << "Argument error: " << e.what() << "\n";
        help = true;
    }

    // Validation
    if ((posy != 3 && posy != 4) || (rosy != 2 && rosy != 4 && rosy != 6)) {
        std::cerr << "Invalid symmetry type.\n";
        help = true;
    }

    int nSizeConstraints = (scale > 0 ? 1 : 0)
                         + (face_count > 0 ? 1 : 0)
                         + (vertex_count > 0 ? 1 : 0);
    if (nSizeConstraints > 1) {
        std::cerr << "Specify at most one of --scale, --faces, --vertices.\n";
        help = true;
    }

    if (input_path.empty() && !help) {
        std::cerr << "No input file specified.\n";
        help = true;
    }

    if (output_path.empty() && !help) {
        std::cerr << "No output file specified (-o).\n";
        help = true;
    }

    if (help) {
        print_usage(argv[0]);
        return -1;
    }

    //tbb::task_scheduler_init tbb_init( nprocs == -1 ? tbb::task_scheduler_init::automatic : nprocs);

    try {
        Mesher mesher(deterministic);

        // Optional: print progress to stdout
        mesher.setProgressCallback([](const std::string &caption, Float value) {
            if (value >= 0.f)
                std::cout << "\r  " << caption << " .. "
                          << int(value * 100) << "%   " << std::flush;
            else
                std::cout << "\r  " << caption << "         " << std::flush;
        });

        mesher.setExtrinsic(extrinsic);
        mesher.setSmoothIterations((int)smooth_iter);
        mesher.setPureQuad(!dominant);           // dominant  mixed; else pure
        mesher.setAlignToBoundaries(align_to_boundaries);

        // 1. Load and build hierarchy
        std::cout << "\n[1/4] Loading input: " << input_path << "\n";
        mesher.loadInput(input_path, crease_angle,
                         scale, face_count, vertex_count,
                         rosy, posy, (int)knn_points);
        std::cout << "\n";

        // 2. Orientation field
        std::cout << "[2/4] Solving orientation field ...\n";
        mesher.solveOrientation();
        std::cout << "\n";

        // 3. Position field
        std::cout << "[3/4] Solving position field ...\n";
        mesher.solvePosition();
        std::cout << "\n";

        // 4. Extract and save
        std::cout << "[4/4] Extracting mesh ...\n";
        mesher.extractMesh();
        mesher.saveOutput(output_path);
        std::cout << "\nDone.\n";

    } catch (const std::exception &e) {
        std::cerr << "\nFatal error: " << e.what() << "\n";
        return -1;
    }

    return EXIT_SUCCESS;
}