#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <string>
#include <optional>
#include <regex>
#include <filesystem>

#pragma push_macro("Handle")
#undef Handle

#include <pxr/pxr.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usd/primRange.h>
#include <pxr/usd/usd/attribute.h>

#include <pxr/usd/sdf/declareHandles.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/sdf/layer.h>
#include <pxr/usd/sdf/changeBlock.h>
#include <pxr/usd/sdf/primSpec.h>
#include <pxr/usd/sdf/namespaceEdit.h>
 
#pragma pop_macro("Handle")

#include "stepAPI.h"
 
#include "Logger.h"
 
PXR_NAMESPACE_USING_DIRECTIVE

namespace fs = std::filesystem;

static std::string getStepLabel(const UsdPrim& prim) {
    TfToken value;
    AutolibStepAPI(prim).CreateStepLabelAttr().Get(&value);
    return value.GetString();
}

static const std::regex kHashSuffixRe(R"(__[0-9a-fA-F]{8}$)");

static std::string stripHashSuffix(const std::string& name) {
    return std::regex_replace(name, kHashSuffixRe, "");
}

static std::string canonicalPath(const SdfPath& path) {
    const std::string pathStr = path.GetString();
    std::string result;
    result.reserve(pathStr.size());

    std::string::size_type start = 0;
    while (start < pathStr.size()) {
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

struct OldPrimInfo {
    SdfPath path;
    std::string stepLabel;
};

/// Index all prims in a stage by their canonical (hash-stripped) path.
/// Maps to a vector because multiple prims might share the same base name.
static std::unordered_map<std::string, std::vector<OldPrimInfo>> buildCanonicalIndex(const UsdStageRefPtr& stage) {
    std::unordered_map<std::string, std::vector<OldPrimInfo>> index;
    for (const UsdPrim& prim : stage->TraverseAll()) {
        const SdfPath path = prim.GetPath();
        if (path == SdfPath::AbsoluteRootPath()) continue;

        std::string cPath = canonicalPath(path);
        std::string label = getStepLabel(prim); // Cache the label for fallback
        
        index[cPath].push_back({path, label});
    }
    return index;
}
/// Pairs a target prim's local SdfPath with the new SdfPath it should be
/// renamed to.  After matching, oldToNew[targetPath] = newAssemblyPath.
struct PathMapping {
    std::unordered_map<SdfPath, SdfPath, SdfPath::Hash> oldToNew;
    std::vector<SdfPath> unmatched;
};

static PathMapping buildPathMapping(
    const UsdStageRefPtr& targetStage,
    const UsdStageRefPtr& oldAssemblyStage,
    const UsdStageRefPtr& newAssemblyStage,
    const SdfPath& prefixPath,
    const std::unordered_set<SdfPath, SdfPath::Hash>& skipPaths
) {
    PathMapping result;

    // index the old assembly by canonical path -> vector of OldPrimInfo
    const auto oldIndex = buildCanonicalIndex(oldAssemblyStage);

    // traverse the new assembly and build oldAssemblyPath -> newAssemblyPath
    std::unordered_map<SdfPath, SdfPath, SdfPath::Hash> assemblyRemap;
    for (const UsdPrim& newPrim : newAssemblyStage->TraverseAll()) {
        if (!newPrim.IsValid()) continue;
        const SdfPath newPath = newPrim.GetPath();
        if (newPath == SdfPath::AbsoluteRootPath()) continue;

        std::string cPath = canonicalPath(newPath);

        auto it = oldIndex.find(cPath);
        if (it == oldIndex.end() || it->second.empty()) {
            LOG_DEBUG("New assembly prim has no match in old assembly for canonical path '" + cPath + "': " + newPath.GetString());
            continue;
        }

        SdfPath matchedOldPath;

        // If there is only one match for the root name, use it.
        // There are multiple prims with the same root name, disambiguate with step:label
        if (it->second.size() == 1) {
            matchedOldPath = it->second.front().path;
        } else {
            const std::string newLabel = getStepLabel(newPrim);
            if (newLabel.empty()) {
                LOG_DEBUG("Collision for '" + cPath + "', but new prim has no step:label to disambiguate. Skipping.");
                continue;
            }

            bool foundMatch = false;
            for (const auto& oldPrimInfo : it->second) {
                if (oldPrimInfo.stepLabel == newLabel) {
                    matchedOldPath = oldPrimInfo.path;
                    foundMatch = true;
                    break;
                }
            }

            if (!foundMatch) {
                LOG_DEBUG("Collision for '" + cPath + "', and no matching step:label found for '" + newLabel + "'. Skipping.");
                continue;
            }
        }

        // Skip pairs where the old path lives under an excluded subtree.
        if (!skipPaths.empty()) {
            bool shouldSkip = false;
            for (const SdfPath& p : matchedOldPath.GetPrefixes()) {
                if (skipPaths.find(p) != skipPaths.end()) {
                    shouldSkip = true;
                    break;
                }
            }
            if (shouldSkip) {
                LOG_DEBUG("Skipping match (excluded prefix): " + matchedOldPath.GetString());
                continue;
            }
        }

        if (matchedOldPath != newPath) {
            assemblyRemap[matchedOldPath] = newPath;
            LOG_DEBUG("Assembly remap matched via " + std::string(it->second.size() > 1 ? "step:label fallback" : "canonical path") + ":\n  " + matchedOldPath.GetString() + "\n  -> " + newPath.GetString());
        }
    }

    // For each prim in the target stage, check whether its prefixed path
    // appears as an old-assembly path in the remap table.
    for (const UsdPrim& targetPrim : targetStage->TraverseAll()) {
        if (!targetPrim.IsValid()) continue;

        const SdfPath localPath = targetPrim.GetPath();
        if (localPath == SdfPath::AbsoluteRootPath()) continue;

        SdfPath lookupPath = localPath;
        if (prefixPath != SdfPath::AbsoluteRootPath()) {
            lookupPath = prefixPath.AppendPath(localPath.MakeRelativePath(SdfPath::AbsoluteRootPath()));
        }

        auto it = assemblyRemap.find(lookupPath);
        if (it == assemblyRemap.end()) {
            LOG_DEBUG("No assembly remap for target prim: " + localPath.GetString());
            result.unmatched.push_back(localPath);
            continue;
        }

        result.oldToNew[localPath] = it->second;
        LOG_DEBUG("Matched target prim:\n  " + localPath.GetString() + "\n  -> " + it->second.GetString());
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
        LOG_PROGRESS(++currentCount, totalJobs, "Editing Names");

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
    "                      matching on the step:label (OCCT TDF label path) primvar.\n"
    " Options:\n"
    "    -r, --oldAssembly       <path>   Path to the old assembly USD stage (old names).\n"
    "    -n, --newAssembly       <path>   Path to the new assembly USD stage (new names).\n"
    "    -t, --target            <path>   Path to the target USD stage to be renamed.\n"
    "    -p, --prefix            <path>   Prefix path for the target stage.\n"
    "    -s, --skip              <path>   Skip any paths in the old assembly that have this path as a prefix. Can be multiple.\n"
    "    -d, --dryRun                     Print the mapping but do not write to disk.\n"
    "    -q, --quiet                      Suppress all output.\n"
    "    -v, --verbose                    Verbose output.\n"
    "    -h, --help                       Print this message.\n\n"
    "    usage: StagePathRemapper -r <oldAssembly.usd> -n <newAssembly.usd> -t <target.usd> [options]\n";

struct RemapperArgs {

    enum ParseResult {
        SUCCESS,
        SUCCESS_CONSUME_NEXT,
        FAILURE,
        EXIT
    };

    fs::path oldAssembly;
    fs::path newAssembly;
    fs::path targetFile;
    SdfPath prefixPath = SdfPath::AbsoluteRootPath();
    std::unordered_set<SdfPath, SdfPath::Hash> skipPaths;
    bool dryRun = false;

    ParseResult parse(const std::string& token, const std::string& next) {
        if (token == "-r" || token == "--oldAssembly") {
            if (next.empty()) {
                std::cerr << "Expected a value after: " << token << "\n";
                return FAILURE;
            }
            if (!oldAssembly.empty()) {
                std::cerr << token << " is already set!\n";
                return FAILURE;
            }
            oldAssembly = next;
            return SUCCESS_CONSUME_NEXT;
        }

        if (token == "-n" || token == "--newAssembly") {
            if (next.empty()) {
                std::cerr << "Expected a value after: " << token << "\n";
                return FAILURE;
            }
            if (!newAssembly.empty()) {
                std::cerr << token << " is already set!\n";
                return FAILURE;
            }
            newAssembly = next;
            return SUCCESS_CONSUME_NEXT;
        }

        if (token == "-t" || token == "--target") {
            if (next.empty()) {
                std::cerr << "Expected a value after: " << token << "\n";
                return FAILURE;
            }
            if (!targetFile.empty()) {
                std::cerr << token << " is already set!\n";
                return FAILURE;
            }
            targetFile = next;
            return SUCCESS_CONSUME_NEXT;
        }

        if (token == "-p" || token == "--prefix") {
            if (next.empty()) {
                std::cerr << "Expected a value after: " << token << "\n";
                return FAILURE;
            }
            prefixPath = SdfPath(next);
            return SUCCESS_CONSUME_NEXT;
        }

        if (token == "-s" || token == "--skip") {
            if (next.empty()) {
                std::cerr << "Expected a value after: " << token << "\n";
                return FAILURE;
            }
            skipPaths.insert(SdfPath(next));
            return SUCCESS_CONSUME_NEXT;
        }

        if (token == "-d" || token == "--dryRun") {
            dryRun = true;
            return SUCCESS;
        }

        if (token == "-q" || token == "--quiet") {
            Logger::activeLevel = Logger::NONE;
            return SUCCESS;
        }

        if (token == "-v" || token == "--verbose") {
            Logger::activeLevel = Logger::DEBUG;
            return SUCCESS;
        }

        if (token == "-h" || token == "--help") {
            std::cout << kArgOptions << std::endl;
            return EXIT;
        }

        std::cerr << "Unrecognized option: " << token << "\n";
        std::cout << kArgOptions << std::endl;
        return FAILURE;
    }

    bool verify() const {
        bool ok = true;
        if (oldAssembly.empty()) {
            std::cerr << "--oldAssembly is required.\n"; 
            ok = false;
        } else if (!fs::exists(oldAssembly)) {
            std::cerr << "Old assembly file not found: " << oldAssembly << "\n"; 
            ok = false;
        }

        if (newAssembly.empty()) {
            std::cerr << "--newAssembly is required.\n"; 
            ok = false;
        } else if (!fs::exists(newAssembly)) {
            std::cerr << "New assembly file not found: " << newAssembly << "\n"; 
            ok = false;
        }

        if (targetFile.empty()) {
            std::cerr << "--target is required.\n"; 
            ok = false;
        } else if (!fs::exists(targetFile)) {
            std::cerr << "Target file not found: " << targetFile << "\n"; 
            ok = false;
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
            case RemapperArgs::SUCCESS: break;
            case RemapperArgs::SUCCESS_CONSUME_NEXT: ++i; break;
            case RemapperArgs::FAILURE: return 1;
            case RemapperArgs::EXIT: return 0;
        }
    }

    if (!args.verify()) return 1;

    auto start = std::chrono::high_resolution_clock::now();

    PathMapping mapping;
    {
        UsdStageRefPtr oldAssemblyStage = UsdStage::Open(args.oldAssembly.string(), UsdStage::LoadNone);
        UsdStageRefPtr newAssemblyStage = UsdStage::Open(args.newAssembly.string(), UsdStage::LoadNone);
        UsdStageRefPtr targetStage      = UsdStage::Open(args.targetFile.string(),  UsdStage::LoadNone);

        mapping = buildPathMapping(targetStage, oldAssemblyStage, newAssemblyStage, args.prefixPath, args.skipPaths);
        LOG_INFO("Found " + std::to_string(mapping.oldToNew.size()) + " paths to rename.");
        if (!mapping.unmatched.empty()) {
            LOG_WARN("Additionally, " + std::to_string(mapping.unmatched.size()) + " paths in the target stage had no match in the reference stage and will be left unchanged.");
            for (const auto& p : mapping.unmatched) {
                LOG_DEBUG("  Unmatched: " + p.GetString());
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