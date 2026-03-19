/*
    main.cpp: CLI entry point for the bare-minimum Instant Meshes pipeline.

    Original work:
        Instant Field-Aligned Meshes
        Wenzel Jakob, Daniele Panozzo, Marco Tarini, and Olga Sorkine-Hornung
        In ACM Transactions on Graphics (Proc. SIGGRAPH Asia 2015)

    All rights reserved. Use of this source code is governed by a
    BSD-style license that can be found in the LICENSE.txt file.
*/

#include <iostream>
#include <cstring>
#include <cstdlib>
#include <filesystem>

#include "instant_meshes/mesher.h"
#include "instant_meshes/common.h"

#include "ArgumentHandler.h"

int nprocs = -1;

const std::string argOptions =
    " Instant Meshes -- Field-aligned mesh remesher\n"
    " Options:\n"
    "   Required:\n"
    "      --output <file>              Output PLY or OBJ path\n"
    "\n"
    "   Sizing (pick at most one):\n"
    "      --scale <length>             Target world-space edge length\n"
    "      --faces <count>              Target face count\n"
    "      --vertices <count>           Target vertex count\n"
    "\n"
    "   Field options:\n"
    "      --rosy <2|4|6>               Orientation symmetry type (default: 4)\n"
    "      --posy <4|6>                 Position symmetry type    (default: 4)\n"
    "      --intrinsic                  Use intrinsic smoothness energy\n"
    "      --boundaries                 Align output edges to mesh boundaries\n"
    "      --dominant                   Dominant (mixed) mesh instead of pure tri/quad\n"
    "\n"
    "   Pre-processing:\n"
    "      --crease <degrees>           Dihedral angle threshold for sharp creases\n"
    "      --smooth <iter>              Post-process smoothing iterations (default: 2)\n"
    "      --knn <count>                Point-cloud: neighbours to consider (default: 10)\n"
    "\n"
    "   Misc:\n"
    "      --threads <count>            Worker thread count (default: automatic)\n"
    "      --deterministic              Prefer slower but deterministic algorithms\n"
    "      --help                       Show this message\n";


struct MeshArgumentHandler {

    MeshParams params;

    enum ParseResult { SUCCESS, SUCCESS_CONSUME_NEXT, FAILURE, EXIT };

    ParseResult parse(const std::string &token, const std::string &nextToken) {

        switch (hashString(token)) {
            case hashString("--help"):
            case hashString("-h"): {
                std::cout << argOptions;
                return EXIT;
            }
            case hashString("--deterministic"):
            case hashString("-d"): {
                params.deterministic = true;
                return SUCCESS;
            }
            case hashString("--intrinsic"):
            case hashString("-i"): {
                params.extrinsic = false;
                return SUCCESS;
            }
            case hashString("--boundaries"):
            case hashString("-b"): {
                params.alignToBoundaries = true;
                return SUCCESS;
            }
            case hashString("--dominant"):
            case hashString("-D"): {
                params.dominant = true;
                return SUCCESS;
            }
            case hashString("--threads"):
            case hashString("-t"): {
                if (nextToken.empty()) goto expectOption;
                nprocs = str_to_uint32_t(nextToken);
                return SUCCESS_CONSUME_NEXT;
            }
            case hashString("--smooth"):
            case hashString("-S"): {
                if (nextToken.empty()) goto expectOption;
                params.smoothIter = str_to_uint32_t(nextToken);
                return SUCCESS_CONSUME_NEXT;
            }
            case hashString("--knn"):
            case hashString("-k"): {
                if (nextToken.empty()) goto expectOption;
                params.knnPoints = str_to_uint32_t(nextToken);
                return SUCCESS_CONSUME_NEXT;
            }
            case hashString("--crease"):
            case hashString("-c"): {
                if (nextToken.empty()) goto expectOption;
                params.creaseAngle = str_to_float(nextToken);
                return SUCCESS_CONSUME_NEXT;
            }
            case hashString("--rosy"):
            case hashString("-r"): {
                if (nextToken.empty()) goto expectOption;
                params.rosy = str_to_int32_t(nextToken);
                return SUCCESS_CONSUME_NEXT;
            }
            case hashString("--posy"):
            case hashString("-p"): {
                if (nextToken.empty()) goto expectOption;
                params.posy = str_to_int32_t(nextToken);
                if (params.posy == 6) params.posy = 3;
                return SUCCESS_CONSUME_NEXT;
            }
            case hashString("--scale"):
            case hashString("-s"): {
                if (nextToken.empty()) goto expectOption;
                params.scale = str_to_float(nextToken);
                return SUCCESS_CONSUME_NEXT;
            }
            case hashString("--faces"):
            case hashString("-f"): {
                if (nextToken.empty()) goto expectOption;
                params.faceCount = str_to_int32_t(nextToken);
                return SUCCESS_CONSUME_NEXT;
            }
            case hashString("--vertices"):
            case hashString("-v"): {
                if (nextToken.empty()) goto expectOption;
                params.vertexCount = str_to_int32_t(nextToken);
                return SUCCESS_CONSUME_NEXT;
            }
            case hashString("--output"):
            case hashString("-o"): {
                if (nextToken.empty()) goto expectOption;
                if (!params.outputPath.empty()) goto alreadySet;
                params.outputPath = nextToken;
                return SUCCESS_CONSUME_NEXT;
            }
            default: {
                if (!token.empty() && token[0] != '-') {
                    if (!params.inputPath.empty()) {
                        std::cerr << "Unexpected positional argument: " << token << "\n";
                        return FAILURE;
                    }
                    params.inputPath = token;
                    return SUCCESS;
                }
                std::cerr << "Unrecognized command-line option: " << token << "\n";
                return FAILURE;
            }
        }

        alreadySet: {
            std::cerr << token << " is already set!\n";
            return FAILURE;
        }
        expectOption: {
            std::cerr << "Expected another token following command-line option: " << token << "\n";
            return FAILURE;
        }
    }

    bool verify() const {
        if (params.inputPath.empty()) {
            std::cerr << "No input file specified.\n";
            return false;
        }
        if (params.outputPath.empty()) {
            std::cerr << "No output file specified (--output).\n";
            return false;
        }
        if ((params.posy != 3 && params.posy != 4) ||
            (params.rosy != 2 && params.rosy != 4 && params.rosy != 6)) {
            std::cerr << "Invalid symmetry type.\n";
            return false;
        }
        int nSizeConstraints = (params.scale       > 0 ? 1 : 0)
                             + (params.faceCount   > 0 ? 1 : 0)
                             + (params.vertexCount > 0 ? 1 : 0);
        if (nSizeConstraints > 1) {
            std::cerr << "Specify at most one of --scale, --faces, --vertices.\n";
            return false;
        }
        return true;
    }
};


int main(int argc, char **argv) {
    std::vector<std::string> tokens;
    for (int i = 1; i < argc; ++i)
        tokens.emplace_back(argv[i]);

    MeshArgumentHandler handler;

    for (size_t i = 0; i < tokens.size(); ++i) {
        const std::string &token     = tokens[i];
        const std::string &nextToken = (i + 1 < tokens.size()) ? tokens[i + 1] : "";

        switch (handler.parse(token, nextToken)) {
            case MeshArgumentHandler::SUCCESS:
                break;
            case MeshArgumentHandler::SUCCESS_CONSUME_NEXT:
                ++i;
                break;
            case MeshArgumentHandler::FAILURE:
                std::cerr << "Run with --help for usage.\n";
                return -1;
            case MeshArgumentHandler::EXIT:
                return 0;
        }
    }

    if (!handler.verify()) {
        std::cerr << "Run with --help for usage.\n";
        return -1;
    }

    try {
        Mesher mesher(handler.params);

        mesher.setProgressCallback([](const std::string &caption, Float value) {
            if (value >= 0.f)
                std::cout << "\r  " << caption << " .. "
                          << int(value * 100) << "%   " << std::flush;
            else
                std::cout << "\r  " << caption << "         " << std::flush;
        });

        std::cout << "\nLoading input: " << handler.params.inputPath << "\n";
        mesher.loadInput();

        std::cout << "Solving orientation field ...\n";
        mesher.solveOrientation();

        std::cout << "Solving position field ...\n";
        mesher.solvePosition();

        std::cout << "Extracting mesh ...\n";
        mesher.extractMesh();
        mesher.saveOutput();
        std::cout << "Done.\n";

    } catch (const std::exception &e) {
        std::cerr << "\nFatal error: " << e.what() << "\n";
        return -1;
    }

    return EXIT_SUCCESS;
}