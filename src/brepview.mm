#import <Cocoa/Cocoa.h>

#include <iostream>
#include <vector>
#include <cmath>
#include <filesystem>
#include <string>

#include <Aspect_DisplayConnection.hxx>
#include <OpenGl_GraphicDriver.hxx>
#include <V3d_Viewer.hxx>
#include <V3d_View.hxx>
#include <AIS_InteractiveContext.hxx>
#include <Cocoa_Window.hxx>
#include <BRepTools.hxx>
#include <TopoDS_Shape.hxx>
#include <BRep_Builder.hxx>
#include <AIS_Shape.hxx>
#include <Standard_Failure.hxx>
#include <TopExp_Explorer.hxx>
#include <Quantity_Color.hxx>
#include <Quantity_NameOfColor.hxx>

const std::string argOptions =
    " brepview -- View a .brep file\n"
    " Options: \n"
    "    -i, --input <path>               Path to the input brep file. \n"
    "    -h, --help                       Prints this message.\n\n"
    "    usage: brepview -i <path> [options] \n"
    "\n";

const std::string usageInfo =
    " Controls once the viewer is open:\n"
    "    Left-drag    rotate\n"
    "    Right-drag   pan\n"
    "    Scroll       zoom\n"
    "    Left-click   select an object\n"
    "    F            fit all\n";

struct BRepViewArgs {

    enum ParseResult {
        SUCCESS,
        SUCCESS_CONSUME_NEXT,
        FAILURE,
        EXIT
    };

    std::filesystem::path inputFile;

    ParseResult parse(const std::string& token, const std::string& nextToken) {
        if (token == "-i" || token == "--input") {
            if (nextToken.empty()) {
                std::cerr << "Expected another token following command-line option: " << token << std::endl;
                return FAILURE;
            }
            if (!inputFile.empty()) {
                std::cerr << token << " is already set!" << std::endl;
                return FAILURE;
            }
            inputFile = nextToken;
            return SUCCESS_CONSUME_NEXT;
        }

        if (token == "-h" || token == "--help") {
            std::cout << argOptions << std::endl;
            return EXIT;
        }
        
        if (!token.empty() && token[0] != '-') {
            if (!inputFile.empty()) {
                std::cerr << "inputFile is already set! Unexpected extra argument: " << token << std::endl;
                return FAILURE;
            }
            inputFile = token;
            return SUCCESS;
        }

        std::cout << "Unrecognized command-line option: " << token << std::endl;
        std::cout << argOptions << std::endl;
        return FAILURE;
    }

    bool verify() const {
        if (inputFile.empty()) {
            std::cerr << "inputFile is not set!" << std::endl;
            return false;
        }
        if (!std::filesystem::exists(inputFile)) {
            std::cerr << "The provided input file does not exist: " << inputFile << std::endl;
            return false;
        }

        return true;
    }
};


static void occtPixelFromViewPoint(NSView *theView, NSPoint thePoint,
                                   int &theX,
                                   int& theY) {
    NSRect aBounds = [theView bounds];
    theX = (int)thePoint.x;
    theY = [theView isFlipped] ? (int)thePoint.y
                                : (int)(aBounds.size.height - thePoint.y);
}

int main(int argc, char** argv) {
    @autoreleasepool {

        std::vector<std::string> tokens;
        for (int i = 1; i < argc; i++) {
            tokens.emplace_back(argv[i]);
        }

        BRepViewArgs inputArgs;
        for (size_t i = 0; i < tokens.size(); i++) {
            const std::string& token = tokens[i];
            const std::string& nextToken = i + 1 < tokens.size() ? tokens[i + 1] : "";

            BRepViewArgs::ParseResult parseResult = inputArgs.parse(token, nextToken);
            switch (parseResult) {
                case BRepViewArgs::SUCCESS:
                    break;
                case BRepViewArgs::SUCCESS_CONSUME_NEXT:
                    i++;
                    break;
                case BRepViewArgs::FAILURE:
                    return 1;
                case BRepViewArgs::EXIT:
                    return 0;
            }
        }

        if (!inputArgs.verify()) {
            std::cerr << "Input argument verification failed." << std::endl;
            return 1;
        }

        [NSApplication sharedApplication];
        [NSApp setActivationPolicy:NSApplicationActivationPolicyRegular];
        [NSApp finishLaunching];

        try {
            Handle(Aspect_DisplayConnection) aDisplayConnection = new Aspect_DisplayConnection();
            Handle(OpenGl_GraphicDriver) aGraphicDriver = new OpenGl_GraphicDriver(aDisplayConnection);

            Handle(V3d_Viewer) aViewer = new V3d_Viewer(aGraphicDriver);
            aViewer->SetDefaultLights();
            aViewer->SetLightOn();

            Handle(AIS_InteractiveContext) myContext = new AIS_InteractiveContext(aViewer);

            Handle(V3d_View) myView = aViewer->CreateView();

            Handle(Cocoa_Window) aWindow = new Cocoa_Window("OCCT BREP Viewer", 0, 0, 800, 600);
            myView->SetWindow(aWindow);
            if (!aWindow->IsMapped()) {
                aWindow->Map();
            }

            [NSApp activateIgnoringOtherApps:true];

            // Read the BREP
            TopoDS_Shape aShape;
            BRep_Builder aBuilder;
            std::filesystem::path abs_path = std::filesystem::absolute(inputArgs.inputFile);
            bool isOK = BRepTools::Read(aShape, abs_path.c_str(), aBuilder);

            if (isOK) {
                std::cout << usageInfo << std::endl;
                
                Handle(AIS_Shape) baseShape = new AIS_Shape(aShape);
                myContext->SetColor(baseShape, Quantity_NOC_GRAY80, Standard_False);
                myContext->SetTransparency(baseShape, 0.7, Standard_False);
                myContext->Display(baseShape, AIS_Shaded, 0, Standard_False);
    
                const Quantity_NameOfColor aColors[] = {
                    Quantity_NOC_RED, Quantity_NOC_GREEN, Quantity_NOC_BLUE1,
                    Quantity_NOC_CYAN1, Quantity_NOC_MAGENTA1, Quantity_NOC_YELLOW,
                    Quantity_NOC_ORANGE, Quantity_NOC_PURPLE, Quantity_NOC_PINK,
                    Quantity_NOC_GOLD
                };
                const int numColors = sizeof(aColors) / sizeof(aColors[0]);
    
                // Extract wires and display each with a unique color and thick line weight
                TopExp_Explorer wireExp(aShape, TopAbs_WIRE);
                int wireIndex = 0;
    
                for (; wireExp.More(); wireExp.Next()) {
                    Handle(AIS_Shape) aisWire = new AIS_Shape(wireExp.Current());
                    Quantity_Color color(aColors[wireIndex % numColors]);
                    
                    myContext->SetColor(aisWire, color, Standard_False);
                    myContext->SetWidth(aisWire, 3.0, Standard_False); // Make wires bold
                    
                    myContext->Display(aisWire, AIS_WireFrame, 0, Standard_False);
                    wireIndex++;
                }
    
                std::cout << "Found and colorized " << wireIndex << " wire(s)." << std::endl;
    
                myView->FitAll();
                myView->Redraw();
            } else {
                std::cerr << "Warning: failed to read shape from " << abs_path << std::endl;
            }

            NSWindow* nsWindow = [[NSApp windows] firstObject];
            NSView* nsContentView = [nsWindow contentView];
            [nsWindow setAcceptsMouseMovedEvents:true];

            // Keep the view in sync with the window's actual size as the
            // user resizes it.
            id resizeObserver =
                [[NSNotificationCenter defaultCenter]
                    addObserverForName:NSWindowDidResizeNotification
                                object:nsWindow
                                 queue:nil
                            usingBlock:^(NSNotification*) {
                                if (!myView.IsNull() && !myView->Window().IsNull()) {
                                    myView->Window()->DoResize();
                                    myView->MustBeResized();
                                    myView->Redraw();
                                }
                            }];

            bool isRotating = false;
            bool isPanning = false;
            NSPoint dragAnchor = NSZeroPoint;

            bool keepRunning = true;
            while (keepRunning) {
                NSEvent* event = [NSApp nextEventMatchingMask:NSEventMaskAny
                                                     untilDate:[NSDate distantFuture]
                                                        inMode:NSDefaultRunLoopMode
                                                       dequeue:true];
                if (event != nil) {
                    if ([event window] == nsWindow) {
                        NSPoint viewPt = [nsContentView convertPoint:[event locationInWindow] fromView:nil];
                        Standard_Integer aX = 0, aY = 0;
                        occtPixelFromViewPoint(nsContentView, viewPt, aX, aY);

                        switch ([event type]) {
                            case NSEventTypeLeftMouseDown: {
                                isRotating = true;
                                dragAnchor = viewPt;
                                myView->StartRotation(aX, aY);
                                break;
                            }
                            case NSEventTypeLeftMouseDragged: {
                                if (isRotating) {
                                    myView->Rotation(aX, aY);
                                    myView->Redraw();
                                }
                                break;
                            }
                            case NSEventTypeLeftMouseUp: {
                                if (isRotating) {
                                    CGFloat dx = viewPt.x - dragAnchor.x;
                                    CGFloat dy = viewPt.y - dragAnchor.y;
                                    if (std::abs(dx) < 3.0 && std::abs(dy) < 3.0) {
                                        // Barely moved -- treat it as a selection click.
                                        myContext->MoveTo(aX, aY, myView, true);
                                        myContext->SelectDetected();
                                        myView->Redraw();
                                    }
                                }
                                isRotating = false;
                                break;
                            }
                            case NSEventTypeRightMouseDown: {
                                isPanning = true;
                                dragAnchor = viewPt;
                                break;
                            }
                            case NSEventTypeRightMouseDragged: {
                                if (isPanning) {
                                    Standard_Integer prevX = 0, prevY = 0;
                                    occtPixelFromViewPoint(nsContentView, dragAnchor, prevX, prevY);
                                    myView->Pan(aX - prevX, -(aY - prevY));
                                    dragAnchor = viewPt;
                                    myView->Redraw();
                                }
                                break;
                            }
                            case NSEventTypeRightMouseUp: {
                                isPanning = false;
                                break;
                            }
                            case NSEventTypeMouseMoved: {
                                if (!isRotating && !isPanning) {
                                    myContext->MoveTo(aX, aY, myView, true);
                                }
                                break;
                            }
                            case NSEventTypeScrollWheel: {
                                Standard_Integer aDelta = (Standard_Integer)([event scrollingDeltaY] * 4.0);
                                myView->StartZoomAtPoint(aX, aY);
                                myView->ZoomAtPoint(0, 0, aDelta, aDelta);
                                myView->Redraw();
                                break;
                            }
                            case NSEventTypeKeyDown: {
                                NSString* chars = [event charactersIgnoringModifiers];
                                if ([chars isEqualToString:@"f"] || [chars isEqualToString:@"F"]) {
                                    myView->FitAll();
                                    myView->Redraw();
                                }
                                break;
                            }
                            default:
                                break;
                        }
                    }
                    [NSApp sendEvent:event];
                }
                if (nsWindow == nil || ![nsWindow isVisible]) {
                    keepRunning = false;
                }
            }

            [[NSNotificationCenter defaultCenter] removeObserver:resizeObserver];
        } catch (const Standard_Failure& e) {
            std::cerr << "OCCT error: " << e.GetMessageString() << std::endl;
            return 1;
        }

        return 0;
    }
}