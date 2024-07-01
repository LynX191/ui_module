import QtQml 2.2
import QtQuick 2.9
import QtQuick.Controls 2.2
import QtGraphicalEffects 1.0
/*



*/
Button{
    id:buttonText
    property int _width:100
    property int  _height:30
    property int _sizeFont: 12
    property color _btnColorClicked:"transparent"
    property color _btnColorDefault:"#141414"
    property color _btnColorMouseOver: "#a673ff"
    property string _text:""
    property string _textColor: "white"
    property string _textFont: "Adobe Gothic Std B" 

    height:_height
    width:_width
    QtObject{
        id: internalButtonText
        property var dynamicColor:  if(buttonText.down){
                                       buttonText.down ? _btnColorClicked : _btnColorDefault
                                    } 
                                    else {
                                        _btnColorDefault
                                    }

    }
    background:Rectangle{
        color: internalButtonText.dynamicColor
        radius: 5
        border{
            width: 3
            color: "#69f7ff"
        }
    }
    contentItem:Text{
        text:_text
        color:_textColor
        font.pointSize:_sizeFont
        font.bold: true
        font.family: _textFont
        verticalAlignment:Text.AlignVCenter
        horizontalAlignment:Text.AlignHCenter    
        wrapMode: Text.Wrap       
    }
}
