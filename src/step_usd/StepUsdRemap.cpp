#include <iostream>
#include <unordered_map>
#include <vector>
#include <string>
#include <string_view>
#include <optional>
#include <regex>
#include <filesystem>

#pragma push_macro("Handle")
#undef Handle

#include <pxr/pxr.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usd/primRange.h>

#include <pxr/usd/sdf/declareHandles.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/sdf/layer.h>
#include <pxr/usd/sdf/changeBlock.h>
#include <pxr/usd/sdf/primSpec.h>
#include <pxr/usd/sdf/namespaceEdit.h>
 
#pragma pop_macro("Handle")

#include "stepContainerAPI.h"
 
#include "ArgumentHandler.h"
#include "Logger.h"
 
PXR_NAMESPACE_USING_DIRECTIVE

static const std::regex kHashSuffixRe(R"(__[0-9a-fA-F]{8}$)");

static std::string stripHashSuffix(const std::string& name) {
    return std::regex_replace(name, kHashSuffixRe, "");
}

static std::string canonicalPath(const SdfPath& path) {
    // Work token-by-token on the string representation.
    // SdfPath elements are separated by '/'; we rebuild the path manually.
    const std::string pathStr = path.GetString();
    std::string result;
    result.reserve(pathStr.size());

    std::string::size_type start = 0;
    while (start < pathStr.size()) {
        // Emit the leading '/' (or the initial empty segment for absolute paths)
        std::string::size_type slash = pathStr.find('/', start);
        std::string::size_type end   = (slash == std::string::npos) ? pathStr.size() : slash;

        std::string token = pathStr.substr(start, end - start);
        result += stripHashSuffix(token);

        if (slash != std::string::npos) {
            result += '/';
            start = slash + 1;
        } else {
            break;
        }
    }
    return result;
}

/// Maps every canonical (hash-stripped) path from the reference stage to
/// the concrete SdfPath that exists there, and separately records the
/// concrete SdfPath from the *target* stage that shares the same canonical
/// path.  After matching, OldPath -> NewPath gives you the rename pairs.
struct PathMapping {
    std::unordered_map<SdfPath, SdfPath, SdfPath::Hash> oldToNew;
    std::vector<SdfPath> unmatched;
};

/// Index all prims in a stage by their canonical path.
static std::unordered_map<std::string, SdfPath> buildCanonicalIndex(const UsdStageRefPtr& stage) {
    std::unordered_map<std::string, SdfPath> index;
    for (const UsdPrim& prim : stage->TraverseAll()) {
        if (!prim.HasAPI<AutolibStepContainerAPI>()) continue;
        
        const SdfPath path = prim.GetPath();
        index[canonicalPath(path)] = path;
    }
    return index;
}

static PathMapping buildPathMapping(const UsdStageRefPtr& targetStage, const UsdStageRefPtr& referenceStage) {
    PathMapping result;

    // Pre-build a canonical -> concrete-path index for the reference stage.
    const auto refIndex = buildCanonicalIndex(referenceStage);

    for (const UsdPrim& targetPrim : targetStage->TraverseAll()) {
        if (!targetPrim.IsValid()) continue;

        const SdfPath targetPath = targetPrim.GetPath();
        if (targetPath == SdfPath::AbsoluteRootPath()) continue;

        const std::string canonical = canonicalPath(targetPath);

        //Fast path: same canonical path exists in reference index
        auto it = refIndex.find(canonical);
        if (it != refIndex.end()) {
            const SdfPath& refPath = it->second;
            if (refPath != targetPath) {
                // Only record if the concrete paths actually differ.
                result.oldToNew[targetPath] = refPath;
            }
            continue;
        }

        // Slow path: canonical paths don't align
        const std::string targetLeafCanonical = stripHashSuffix(targetPath.GetName());
        const std::string targetParentCanonical = canonicalPath(targetPath.GetParentPath());

        bool found = false;
        for (const UsdPrim& refPrim : referenceStage->TraverseAll()) {
            if (!refPrim.IsValid()) continue;
            const SdfPath refPath = refPrim.GetPath();
            if (refPath == SdfPath::AbsoluteRootPath()) continue;

            // Parent canonical must match.
            if (canonicalPath(refPath.GetParentPath()) != targetParentCanonical)
                continue;

            // Stripped leaf name must match.
            if (stripHashSuffix(refPath.GetName()) != targetLeafCanonical)
                continue;

            // Found a candidate.
            if (refPath != targetPath)
                result.oldToNew[targetPath] = refPath;

            found = true;
            break;
        }

        if (!found) {
            result.unmatched.push_back(targetPath);
        }
    }

    return result;
}

static void recursivePatchSpecs(
    const SdfPrimSpecHandle& spec, 
    const std::unordered_map<std::string, std::string>& stringMap,
    int& processedCount,
    int totalSpecs
) {
    if (!spec) return;

    int current = ++processedCount;
    LOG_PROGRESS(current, totalSpecs, "Patching Composition Arcs");

    // References
    if (spec->HasInfo(SdfFieldKeys->References)) {
        SdfReferenceListOp refListOp = spec->GetInfo(SdfFieldKeys->References).Get<SdfReferenceListOp>();
        bool modified = false;

        refListOp.ModifyOperations([&](const SdfReference& ref) -> std::optional<SdfReference> {
            const std::string targetPath = ref.GetPrimPath().GetString();
            
            auto it = stringMap.find(targetPath);
            if (it != stringMap.end()) {
                modified = true;
                LOG_DEBUG("Reference Patch: " + targetPath + "  ->  " + it->second);
                
                return SdfReference(
                    ref.GetAssetPath(),
                    SdfPath(it->second),
                    ref.GetLayerOffset(),
                    ref.GetCustomData()
                );
            }
            return ref;
        });

        if (modified) {
            spec->SetInfo(SdfFieldKeys->References, VtValue(refListOp));
        }
    }

    // Payloads
    if (spec->HasInfo(SdfFieldKeys->Payload)) {
        SdfPayloadListOp payloadListOp = spec->GetInfo(SdfFieldKeys->Payload).Get<SdfPayloadListOp>();
        bool modified = false;

        payloadListOp.ModifyOperations([&](const SdfPayload& ref) -> std::optional<SdfPayload> {
            const std::string targetPath = ref.GetPrimPath().GetString();
            
            auto it = stringMap.find(targetPath);
            if (it != stringMap.end()) {
                modified = true;
                LOG_DEBUG("Payload Patch: " + targetPath + "  ->  " + it->second);
                
                return SdfPayload(
                    ref.GetAssetPath(),
                    SdfPath(it->second),
                    ref.GetLayerOffset()
                );
            }
            return ref;
        });

        if (modified) {
            spec->SetInfo(SdfFieldKeys->Payload, VtValue(payloadListOp));
        }
    }

    // Inherits
    if (spec->HasInfo(SdfFieldKeys->InheritPaths)) {
        SdfPathListOp inheritListOp = spec->GetInfo(SdfFieldKeys->InheritPaths).Get<SdfPathListOp>();
        bool modified = false;

        inheritListOp.ModifyOperations([&](const SdfPath& path) -> std::optional<SdfPath> {
            const std::string targetPath = path.GetString();

            auto it = stringMap.find(targetPath);
            if (it != stringMap.end()) {
                modified = true;
                LOG_DEBUG("Inherit Patch: " + targetPath + "  ->  " + it->second);
                
                return SdfPath(it->second);
            }
            return path;
        });

        if (modified) {
            spec->SetInfo(SdfFieldKeys->InheritPaths, VtValue(inheritListOp));
        }
    }

    for (const SdfPrimSpecHandle& child : spec->GetNameChildren()) {
        recursivePatchSpecs(child, stringMap, processedCount, totalSpecs);
    }
}

static void patchCompositionArcs(
    const SdfLayerRefPtr& layer, 
    const PathMapping& mapping
) {
    if (!layer) return;

    std::unordered_map<std::string, std::string> stringMap;
    stringMap.reserve(mapping.oldToNew.size());
    for (const auto& [oldPath, newPath] : mapping.oldToNew) {
        stringMap[oldPath.GetString()] = newPath.GetString();
    }

    if (stringMap.empty()) {
        LOG_DEBUG("No paths to remap in reference arcs.");
        return;
    }

    int totalSpecs = 0;
    std::function<void(const SdfPrimSpecHandle&)> countSpecs = [&](const SdfPrimSpecHandle& s) {
        if (!s) return;
        totalSpecs++;
        for (const auto& child : s->GetNameChildren()) countSpecs(child);
    };

    countSpecs(layer->GetPseudoRoot());

    int processedCount = 0;
    recursivePatchSpecs(layer->GetPseudoRoot(), stringMap, processedCount, totalSpecs);
    
    LOG_PROGRESS_DONE();
    LOG_DEBUG("Finished patching " + std::to_string(totalSpecs) + " prim specs.");
}

static void applyPathMapping(const SdfLayerRefPtr& targetLayer, const PathMapping& mapping) {
    if (mapping.oldToNew.empty()) {
        LOG_DEBUG("No path differences found, so there is nothing to rename.");
        return;
    }

    // Sort deepest paths first so children are renamed before their parents.
    std::vector<std::pair<SdfPath, SdfPath>> sorted(mapping.oldToNew.begin(), mapping.oldToNew.end());
    std::sort(
        sorted.begin(), 
        sorted.end(),
        [](const auto& a, const auto& b) {
            return a.first.GetPathElementCount() > b.first.GetPathElementCount();
        }
    );

    SdfBatchNamespaceEdit batch;

    int currentCount = 0;
    int totalJobs = (int)sorted.size();

    for (const auto& [oldPath, newPath] : sorted) {
        const SdfPath oldPrimPath = oldPath.GetPrimPath();
        const SdfPath newPrimPath = newPath.GetPrimPath();

        if (!targetLayer->HasSpec(oldPrimPath)) {
            LOG_INFO("Spec not in root layer, skipping: " + oldPrimPath.GetString());
            ++currentCount;
            continue;
        }

        LOG_DEBUG("Queuing rename:\n  " + oldPrimPath.GetString() + "\n  " + newPrimPath.GetString());
        LOG_PROGRESS(++currentCount, totalJobs, "Tessellating Geometry");

        const TfToken newName = newPrimPath.GetNameToken(); // we are only updating the leaves first
        batch.Add(SdfNamespaceEdit::Rename(oldPrimPath, newName));
    }

    LOG_PROGRESS_DONE();

    // Validate before committing - CanApply fills in human-readable reasons.
    SdfNamespaceEditDetailVector details;
    if (!targetLayer->CanApply(batch, &details)) {
        LOG_ERR("SdfBatchNamespaceEdit cannot be applied:");
        for (const auto& d : details) {
            LOG_ERR("  " + d.reason);
        }
        return;
    }

    if (!targetLayer->Apply(batch)) {
        LOG_ERR("SdfBatchNamespaceEdit::Apply failed.");
        return;
    }

    patchCompositionArcs(targetLayer, mapping);

    targetLayer->Save();
}

const std::string kArgOptions =
    " StagePathRemapper -- Renames prims in a target USD stage to match\n"
    "                      the names found in a reference USD stage,\n"
    "                      matching on hash-stripped canonical paths.\n"
    " Options:\n"
    "    -r, --reference <path>   Path to the reference USD stage (new names).\n"
    "    -t, --target    <path>   Path to the target USD stage  (old names).\n"
    "    -d, --dry-run            Print the mapping but do not write to disk.\n"
    "    -q, --quiet              Suppress all output.\n"
    "    -v, --verbose            Verbose output.\n"
    "    -h, --help               Print this message.\n\n"
    "    usage: StagePathRemapper -r <ref.usd> -t <target.usd> [options]\n";

struct RemapperArgs : public ArgumentHandler {
    std::filesystem::path referenceFile;
    std::filesystem::path targetFile;
    bool dryRun = false;

    ParseResult parse(const std::string& token, const std::string& next) override {
        switch (hashString(token)) {
            case hashString("-r"):
            case hashString("--reference"):
                if (next.empty()) goto expectOption;
                if (!referenceFile.empty()) goto alreadySet;
                referenceFile = next;
                return SUCCESS_CONSUME_NEXT;

            case hashString("-t"):
            case hashString("--target"):
                if (next.empty()) goto expectOption;
                if (!targetFile.empty()) goto alreadySet;
                targetFile = next;
                return SUCCESS_CONSUME_NEXT;

            case hashString("-d"):
            case hashString("--dry-run"):
                dryRun = true;
                return SUCCESS;

            case hashString("-q"):
            case hashString("--quiet"):
                Logger::activeLevel = Logger::NONE;
                return SUCCESS;

            case hashString("-v"):
            case hashString("--verbose"):
                Logger::activeLevel = Logger::DEBUG;
                return SUCCESS;

            case hashString("-h"):
            case hashString("--help"):
                std::cout << kArgOptions << std::endl;
                return EXIT;

            default:
                std::cerr << "Unrecognized option: " << token << "\n";
                std::cout << kArgOptions << std::endl;
                return FAILURE;
        }

        alreadySet:
            std::cerr << token << " is already set!\n";
            return FAILURE;
        expectOption:
            std::cerr << "Expected a value after: " << token << "\n";
            return FAILURE;
    }

    bool verify() const override {
        bool ok = true;
        if (referenceFile.empty()) {
            std::cerr << "--reference is required.\n"; ok = false;
        } else if (!std::filesystem::exists(referenceFile)) {
            std::cerr << "Reference file not found: " << referenceFile << "\n"; ok = false;
        }
        if (targetFile.empty()) {
            std::cerr << "--target is required.\n"; ok = false;
        } else if (!std::filesystem::exists(targetFile)) {
            std::cerr << "Target file not found: " << targetFile << "\n"; ok = false;
        }
        return ok;
    }
};

int main(int argc, char** argv) {
    std::vector<std::string> tokens;
    for (int i = 1; i < argc; i++) tokens.emplace_back(argv[i]);

    RemapperArgs args;
    for (size_t i = 0; i < tokens.size(); i++) {
        const std::string& tok  = tokens[i];
        const std::string& next = (i + 1 < tokens.size()) ? tokens[i + 1] : "";

        switch (args.parse(tok, next)) {
            case ArgumentHandler::SUCCESS: break;
            case ArgumentHandler::SUCCESS_CONSUME_NEXT: ++i; break;
            case ArgumentHandler::FAILURE: return 1;
            case ArgumentHandler::EXIT: return 0;
        }
    }

    if (!args.verify()) return 1;

    auto start = std::chrono::high_resolution_clock::now();

    PathMapping mapping;
    {
        UsdStageRefPtr referenceStage = UsdStage::Open(args.referenceFile.string(), UsdStage::LoadNone);
        // Open target via SdfLayer to ensure we have a handle to it later
        SdfLayerRefPtr targetLayer = SdfLayer::FindOrOpen(args.targetFile.string());
        UsdStageRefPtr targetStage = UsdStage::Open(targetLayer, UsdStage::LoadNone);

        mapping = buildPathMapping(targetStage, referenceStage);
        LOG_INFO("Found " + std::to_string(mapping.oldToNew.size()) + " paths to rename.");
        if (!mapping.unmatched.empty()) {
            LOG_WARN("Additionally, " + std::to_string(mapping.unmatched.size()) + " paths in the target stage had no match in the reference stage and will be left unchanged.");
            for (const auto& p : mapping.unmatched) {
                LOG_WARN("  Unmatched: " + p.GetString());
            }
        }
    }

    if (!args.dryRun) {
        SdfLayerRefPtr editLayer = SdfLayer::FindOrOpen(args.targetFile.string());
        
        LOG_DEBUG("Applying renames to target layer...");
        
        {
            SdfChangeBlock block;
            applyPathMapping(editLayer, mapping);
        }
        
        editLayer->Save();
        auto end = std::chrono::high_resolution_clock::now();
        LOG_INFO("Total Time Taken: " + std::to_string(std::chrono::duration<double>(end - start).count()) + " seconds");
    }

    return 0;
}