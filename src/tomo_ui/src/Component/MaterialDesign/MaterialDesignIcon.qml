import QtQuick 2.9
import QtQml 2.0
import "MaterialDesignIconGlyphs.js" as MaterialGlyphs

Item {
    property int size: 24
    property string name: ""
    property color color: "#9E9E9E"

    width: size
    height: size

    Text {
        anchors.fill: parent

        color: parent.color

        font.family: materialFont.name
        font.pixelSize: parent.height

        text: MaterialGlyphs.glyphs[parent.name]
    }

    FontLoader {
        id: materialFont
        source: "materialdesignicons-webfont.ttf"
    }
    Component.onCompleted: {
    }
}
