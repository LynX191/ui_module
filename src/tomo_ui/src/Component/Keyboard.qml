import QtQuick 2.8
import QtQuick.Window 2.3
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3
Popup
{
    id:keyboardPopup
    width: 350
    height: 200
    signal enterKeyNumPress()
    Rectangle
    {
        id:bottomButtons
        focus:true
        anchors.fill: parent
        color: "black"
        Component.onCompleted:
        {
            buttonList.model.append({buttonsText: "7"})
            buttonList.model.append({buttonsText: "8"})
            buttonList.model.append({buttonsText: "9"})

            buttonList.model.append({buttonsText: "4"})
            buttonList.model.append({buttonsText: "5"})
            buttonList.model.append({buttonsText: "6"})

            buttonList.model.append({buttonsText: "1"})
            buttonList.model.append({buttonsText: "2"})
            buttonList.model.append({buttonsText: "3"})

            buttonList.model.append({buttonsText: "+"})
            buttonList.model.append({buttonsText: "0"})
            buttonList.model.append({buttonsText: "-"})

            buttonList.model.append({buttonsText: "Del"})
            buttonList.model.append({buttonsText: "C"})
            buttonList.model.append({buttonsText: "Ent"})
        }
        GridView{
            id:buttonList
            width: 350
            height: 200
            anchors{
                top:parent.top
                bottom:parent.bottom
                left:parent.left
                right:parent.right
            }
            interactive:false
            model:ListModel{}
            cellWidth: parent.width/3
            cellHeight: parent.height/5
            delegate:Button{
                id:buttonKey
                focus:true
                width:bottomButtons.width/3
                height:bottomButtons.height/5
                onClicked:{
                    loginVM.updateIdleTimer()
                    if(buttonText.text == "Del"){
                        delClicked()
                    }
                    else if (buttonText.text == "C"){
                        inputField.text=""
                    }
                    else if (buttonText.text == "Ent"){
                        keyboardPopup.enterKeyNumPress()
                        keyboardPopup.close()
                    }
                    else if (buttonText.text == "0"){
                        num0Clicked()
                    }
                    else if (buttonText.text == "+"){
                        plusClicked()
                    }
                    else if (buttonText.text == "-"){
                        minusClicked()
                    }
                    else{
                        numKeyClicked(buttonText.text)
                    }
                }
                Text{
                    id:buttonText
                    text:buttonsText
                    anchors.centerIn:parent
                    font.pointSize: 16
                    color:buttonKey.down ? "gray" : "white"
                }
                background: Rectangle{
                    color: buttonKey.hovered ? buttonKey.down ? "#26282e" : "#323232" : "#3b3b3b"
                    radius: 7
                    border.color: "black"
                    border.width: 1.5
                }
            }
        }
    }
    function numKeyClicked(key){
        if(inputField.text===""||inputField.text==="0"){
            inputField.text=key
        }
        else{
            inputField=insert(inputField.cursorPosition,key)
        }
    }
    function num0Clicked(key){
        if(inputField.text===""||inputField.text==="0"){
            inputField.text="0"
        }
        else if(inputField.text==="-0"){
            return
        }
        else{
            inputField=insert(inputField.cursorPosition,"0")
        }
    }
    function delClicked(){
        if(inputField.text==="")
            return
        else{
            inputField.remove(inputField.cursorPosition,inputField.cursorPosition-1)
        }
    }
    function plusClicked(){
        if(inputField.text.search("-")===-1){
            return
        }else{
            inputField.text=inputField.text.slice(1)
        }
    }
    function minusClicked(){
        if(inputField.text.search("-")!==-1){
            return
        }else{
            inputField.text="-"+inputField.text
        }
    }

}

