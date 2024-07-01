import QtQml 2.0
import QtQuick 2.9
import QtQuick.Controls 2.2
import QtGraphicalEffects 1.0
import QtQuick.Window 2.2
Window {
    id: rootWindow
    width: 896
    height: 375
    visible: true
    flags: Qt.SplashScreen | Qt.WindowStaysOnTopHint

    signal timeOut()
//    property int _width:width
//    property int _height:height
    property Gradient _borderGradient: borderGradient
    property int _borderWidth: 1
    property double _radius: 0

    property string loadingDescription: "Loading..."
    property string version: "Version: ST02-00.00.00- Copyright \u00A9 EmageTomO Pte Ltd"
    property double loadingValue: 0
    Rectangle {
        visible: parent.visible
        Gradient {
            id: borderGradient
            //        orientation : Gradient.Horizontal
            GradientStop {
                position: 0.000
                color: "#100C07"
            }
            GradientStop {
                position: 0.167
                color: "#3E3B39"
            }
            GradientStop {
                position: 0.333
                color: "#6D6A6A"
            }
            GradientStop {
                position: 0.500
                color:"#F8F8FF"
            }
            GradientStop {
                position: 0.667
                color: "#6D6A6A"
            }
            GradientStop {
                position: 0.833
                color: "#3E3B39"
            }
            GradientStop {
                position: 1.000
                color: "#100C07"
            }
        }
        Loader {
            id: loader
            active: borderGradient
            anchors.fill: parent
            sourceComponent: border
            visible: status == Loader.Ready
        }
        Component.onCompleted: loader.active
        Component {
            id: border
            Item {
                Rectangle {
                    id: borderFill
                    radius: _radius
                    anchors.fill: parent
                    gradient: _borderGradient
                    visible: false
                }

                Rectangle {


                    id: mask
                    radius: _radius
                    border.width: _borderWidth
                    anchors.fill: parent
                    color: 'transparent'
                    visible: false   // otherwise a thin border might be seen.
                }

                OpacityMask {
                    id: opM
                    anchors.fill: parent
                    source: borderFill
                    maskSource: mask
                }
            }
        }

        width: parent.width
        height: parent.height
        clip: true
        color: "#000000"

        layer.enabled: true
        layer.effect: Glow {
            samples: 15
            transparentBorder: true
            cached: true
            radius: 3
            spread:0.5
        }

        Image { // background
            source: "../../Resources/splash_screen_bg.jpg"
            width: 796
            height: 375 - _borderWidth*2
            x:_borderWidth
            y: x
            mipmap: true
        }

        Text { // version
            width: parent.width*0.4
            height: parent.height*0.2
            font.pointSize: 18
            fontSizeMode: Text.Fit
            color: "white"
            text: version
            font.bold: false
            font.family: webFont_4.name
            FontLoader { id: webFont_4; source: "../../Fonts/Dongle-Regular.ttf" }
            anchors.bottom: parent.bottom
            anchors.right: parent.right
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignBottom
            LinearGradient {
                enabled:true
                visible: enabled
                anchors.fill: parent
                source: parent
                antialiasing: true
                gradient: Gradient {
                    GradientStop { position: 0; color: "#1c92d2"}
                    GradientStop { position: 1; color: "#f2fcfe"}
                }
                ShaderEffectSource{
                    recursive: true
                }
            }
        }

        Column{
            width: parent.width*0.55
            height: parent.height*0.7
            anchors.right: parent.right
            anchors.bottom: parent.bottom
            spacing: 15

            Text { // title loading
                text: qsTr("TomO Cartoner")
                color: "#52c234"
                font.pointSize: 30
                font.family: webFont_1.name
                anchors.horizontalCenter: parent.horizontalCenter
                layer.enabled: true
                layer.effect: DropShadow {
                    verticalOffset: 2
                    color: "black"
                    radius: 1
                    samples: 3
                    cached: true
                    spread :0.5
                }
                LinearGradient {
                    enabled:true
                    visible: enabled
                    anchors.fill: parent
                    source: parent
                    antialiasing: true
                    gradient: Gradient {
                        GradientStop { position: 0; color: "#bc4e9c"}
                        GradientStop { position: 1; color: "#f80759"}
                    }
                    ShaderEffectSource{
                        recursive: true
                    }
                }
                FontLoader { id: webFont_1; source: "../../Fonts/Rajdhani-Medium.ttf" }
            }

            Rectangle{
                width: parent.width
                height: parent.height*0.18
                color: "transparent"
                anchors.horizontalCenter: parent.horizontalCenter

                Text { // description when loading
                    text: loadingDescription
                    color: "#19a2a7"
                    font.pointSize: 14
                    font.family: webFont_2.name
                    anchors.centerIn: parent
                    FontLoader { id: webFont_2; source: "../../Fonts/Dongle-Light.ttf" }
                }
            }

            Rectangle{
                id: loadingProgress
                width: 400
                height: 6
                color: "white"
                anchors.horizontalCenter: parent.horizontalCenter
                property color loadColor: "chartreuse"
                Item{
                    clip: true
                    anchors.fill: parent
                    Rectangle{
                        width: parent.width*(loadingValue + 8) / 100
                        height: parent.height
                        color: loadingProgress.loadColor
                        clip: true

                        Behavior on width {
                            SpringAnimation { spring: 2; damping : 0.5}
                        }
                    }
                }


                Text { // value loading %
                    x: parent.width - contentWidth
                    y: parent.height + 3
                    text: parseInt(loadingValue) + "%"
                    color:  loadingProgress.loadColor
                    font.pointSize: 18
                    font.bold: true
                    font.family: webFont_3.name
                    FontLoader { id: webFont_3; source: "../../Fonts/Dongle-Bold.ttf" }
                }
            }
        }
    }

    Timer{
        id: randomTimer
        property var stupidWord : ["\\","|","/","--","\\","|","/","--","\\","|","/","--"]
        property int loadingNumber
        interval: 200
        running: true
        repeat: true
        onTriggered: {
			var randomValue = Math.random()*20
			if(loadingValue + randomValue > 100){
				loadingValue = 100.0
				randomTimer.running = false
                closeTimer.running = true
			}
			else{
            	loadingValue += randomValue
            	loadingDescription = randomTimer.stupidWord[parseInt(Math.random()*1000%11)]
			}
        }
    }
    Timer{
        id: closeTimer
        interval: 300
        running: false
        repeat: false
        onTriggered : {
            rootWindow.timeOut()
            rootWindow.close()
        }
    }
}
