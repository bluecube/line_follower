import adsk.core, adsk.fusion, adsk.cam
import traceback
import sys
import os

def all_subcomponents(component):
    """ Return a name:component dict of all subcomponents (not occurences) of a given component """
    return {occurence.component.name: occurence.component for occurence in component.allOccurrences}


def export_stl(body, export_manager, full_path):
    export_options = export_manager.createSTLExportOptions(body, full_path)
    export_manager.execute(export_options)

    
def export_step(component, export_manager, full_path):
    """ Export the complete component as an overview step file """
    export_options = export_manager.createSTEPExportOptions(full_path, component)
    export_manager.execute(export_options)


def export_screenshot(component, export_manager, full_path):
    view = adsk.core.Application.get().activeViewport # We're hijacking the current viewport :-(
    old_camera = view.camera
    camera = view.camera # This gets a copy of the camera again

    camera.cameraType = adsk.core.CameraTypes.PerspectiveCameraType
    camera.viewOrientation = adsk.core.ViewOrientations.IsoTopRightViewOrientation
    camera.isSmoothTransition = False

    try:
        view.camera = camera
        view.fit()
        camera = view.camera
        camera.viewExtents *= 0.75 # Tighter framing than the default
        view.camera = camera
        view.refresh()
        view.saveAsImageFile(full_path, 1024, 768)
    finally:
        view.camera = old_camera
        pass

    
def collect_visibility(component):
    occurences = {}
    bodies = {}
    for occurence in component.allOccurrences:

        occurences[occurence.name] = occurence.isLightBulbOn

        if occurence.component.name in bodies:
            continue

        bodies_visibility = {}
        for body in occurence.component.bRepBodies:
            bodies_visibility[body.name] = body.isLightBulbOn
        bodies[occurence.component.name] = bodies_visibility

    return occurences, bodies


def restore_visibility(component, visibility_status):
    occurences, bodies = visibility_status
    visited_components = set()

    for occurence in component.allOccurrences:
        occurence.isLightBulbOn = occurences[occurence.name]

        if occurence.component.name in visited_components:
            continue

        visited_components.add(occurence.component.name)
        bodies_visibility = bodies[occurence.component.name]
        for body in occurence.component.bRepBodies:
            if body.name in bodies_visibility:
                body.isLightBulbOn = bodies_visibility[body.name]

                
def default_visibility(component):
    """ Make all non simulation components and bodies visible """
    for occurence in component.allOccurrences:
        occurence.isLightBulbOn =  not occurence.name.upper().startswith("SIM")

        for body in occurence.component.bRepBodies:
            body.isLightBulbOn = not body.name.upper().startswith("SIM")


def collect_jobs(design, export_path):
    export_manager = design.exportManager
    visibility_backup = [None]

    def export_helper(geometry, exporter, basename, extension):
        filename = basename + "." + extension
        full_path = os.path.join(export_path, filename)
        os.makedirs(os.path.dirname(full_path), exist_ok=True)

        def export():
            exporter(geometry, export_manager, full_path)

        return (export, "Exporting " + os.path.basename(filename))

    def backup_visibility():
        visibility_backup[0] = collect_visibility(design.rootComponent)

    jobs = [
        (backup_visibility, "Backing up visibility"),
        (lambda: default_visibility(design.rootComponent), "Setting default visibility"),
        export_helper(design.rootComponent, export_step, design.rootComponent.partNumber, "step"),
        export_helper(design.rootComponent, export_screenshot, design.rootComponent.partNumber, "png"),
    ]
    for name, component in all_subcomponents(design.rootComponent).items():
        for body in component.bRepBodies:
            if body.name.upper().startswith("FDM"):
                jobs.append(export_helper(body, export_stl, body.name, "stl"))
            elif body.name.upper().startswith("SIM"):
                jobs.append(export_helper(body, export_stl, body.name, "stl"))
                #jobs.append(export_helper(body, export_obj, body.name, "obj"))
            pass

    jobs.append(((lambda: restore_visibility(design.rootComponent, visibility_backup[0])), "Restoring visibility"))
    return jobs


def run_jobs(jobs, ui):
    progress  = ui.createProgressDialog()
    progress.cancelButtonText = 'Cancel'
    progress.isBackgroundTranslucent = True
    progress.isCancelButtonShown = True

    progress.show('Exporting', '', 0, len(jobs))
    try:
        for i, job in enumerate(jobs):
            if progress.wasCancelled:
                break
            fun, title = job
            progress.message = "{}; {}/{}".format(title, i + 1, len(jobs))
            progress.progressValue = i

            fun()
    finally:
        progress.hide()


def get_export_path(ui):
    dialog = ui.createFolderDialog()
    dialog.title = "Choose directory for exports"
    result = dialog.showDialog()

    if result != adsk.core.DialogResults.DialogOK:
        return None
    else:
        return dialog.folder


def run(context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    ui.activeSelections.clear()

    try:
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox('No active Fusion 360 design', 'No Design')
            return
        
        export_path = get_export_path(ui)

        if export_path:
            jobs = collect_jobs(design, export_path)
            run_jobs(jobs, ui)

            ui.messageBox(
                "Succesfully exported {} files. Don't forget to manually export sheet metal DXFs and overall f3z.".format(len(jobs)),
                "Done"
            )

    except Exception as e:
        #ui.messageBox("".join(traceback.format_exception(*sys.exc_info())), "Exception")
        raise