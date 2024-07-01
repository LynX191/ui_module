import QtQuick 2.9
import QtQuick.Dialogs 1.2

FileDialog{
    id: file_dialog
    property string _pathInit: ""
    property var _extension: null
    title: qsTr("Select File")
    nameFilters: _extension
    selectExisting: true
    selectFolder: false
    selectMultiple: false
    folder: (_pathInit === "" || _pathInit === undefined) ? shortcuts.desktop: _pathInit
    Component.onCompleted: {
        file_dialog.x = (1920 - file_dialog.width) *0.5
        file_dialog.y = (1080 - file_dialog.height) *0.5
    }
}
