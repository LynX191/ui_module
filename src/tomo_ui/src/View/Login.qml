import QtQml 2.2
import QtQuick 2.9
import QtQuick.Window 2.3
import QtQuick.Dialogs 1.2
import QtQuick.Layouts 1.3
import QtQuick.Controls 1.4 as QQC14
import QtQuick.Controls 2.2
import QtQuick.VirtualKeyboard 2.2
import QtQuick.VirtualKeyboard.Settings 2.2


//My import
import DataLogin 1.0
import "../Component"
import "../Dialog"
import "../Component"
import "../Component/MaterialDesign"
//end

Item{
    id:thisItem
    property bool _clearTextField: false
    property bool _clearTextAfterLogout: false
    property string  color_function_item:"darkgray"
    property string _username: ""
    property string _level: ""
    property string _currentPw: ""
    property int _width:0
    property int _height:0
    property int _fontsize_funtion_view: 16
    property int _fontsize_header_view: 18
    property int _fontsize_inforLogin_view: 16
    property int _fontPixelSize: 28
    property int _fontSubSize: 20
    property string _fontFamily: "Arial"
    property int _codeLoginCheck: 0
    property int _codeChangePasswordCheck: 0
    property int _codeCreateAccountCheck: 0
    property int _codeDeleteAccountCheck: 0
    property int _codeLogoutCheck: 0
    property bool adminBtnIsOf: false
    property bool technicianIsOf: false
    property bool operatorIsOf: false

    width:_width
    height:_height
	signal quitSignal()
    signal viewKeyboardOnInit()

	Connections{
		target: productionVM
		ignoreUnknownSignals: true
		onEnabledForRunLotChanged:{
			var boolValue = (thisItem._level !== "") && (thisItem._username !== "");
			var isAdmin = (thisItem._username === "Admin") && productionVM.enabledForRunLot ;
			var enableList = [!boolValue, isAdmin, boolValue, isAdmin, boolValue];
			internalLoginFunction.changeEnableSelectMode(enableList);
		}
	}
    QtObject{
        id: internalLoginFunction

		function changeEnableSelectMode(enableList){
			for(var i=0; i < listModelFunction.count; i++)
			{
				listModelFunction.setProperty(i, "enableSelect", enableList[i])
			}
		}

        function checkAccountCreate(userT, passwordT, passwordconfirmT, levelBool, levelString) {
            _clearTextField = false;
            var messNewUserDialogContent=""
            var messNewUserDialogDetail=""

            if (_username === "" || _level === "") {
                messNewUserDialogContent = "Account Login Required";
                messNewUserDialogDetail = "Please log in to your account before creating a new one.";
                _clearTextField = true;
            } else if (userT === "") {
                messNewUserDialogContent = "Username Required";
                messNewUserDialogDetail = "Username cannot be empty. Please provide a valid username.";
                _clearTextField = false;
            } else if (passwordT === "" || passwordconfirmT === "") {
                messNewUserDialogContent = "Password Missing";
                messNewUserDialogDetail = "Both password and password confirmation fields are required.";
                _clearTextField = false;
            } else if (passwordT !== passwordconfirmT) {
                messNewUserDialogContent = "Passwords Do Not Match";
                messNewUserDialogDetail = "The password you entered does not match the confirmed password.";
                _clearTextField = false;
            } else if (!levelBool || levelString === "") {
                messNewUserDialogContent = "Access Level Required";
                messNewUserDialogDetail = "Please select an access level for the new account.";
                _clearTextField = false;
            } else {
                loginVM.checkAccountCreateInDatabase(userT, passwordT, levelString);
                if (_codeCreateAccountCheck === 0) {
                    messNewUserDialogContent = "Account Created Successfully";
                    messNewUserDialogDetail = "The user account has been created successfully.";
                    _clearTextField = true;
                } else if (_codeCreateAccountCheck === 1) {
                    messNewUserDialogContent = "User Already Exists";
                    messNewUserDialogDetail = "An account with this username already exists in the database.";
                    _clearTextField = false;
                } else if (_codeCreateAccountCheck === 2) {
                    messNewUserDialogContent = "Access Level Limit Exceeded";
                    messNewUserDialogDetail = "The access level you selected exceeds your account's login level.";
                    _clearTextField = false;
                } else if (_codeCreateAccountCheck === 3) {
                    messNewUserDialogContent = "Invalid Password";
                    messNewUserDialogDetail = "Passwords must have at least 8 characters and contain both uppercase and lowercase letters.";
                    _clearTextField = false;
                } else if (_codeCreateAccountCheck === 4) {
                    messNewUserDialogContent = "Cannot create account";
                    messNewUserDialogDetail = "Exceed limitation for number of Administrator account (1 account)";
                    _clearTextField = false;
                } else if (_codeCreateAccountCheck === 5) {
                    messNewUserDialogContent = "Cannot create account";
                    messNewUserDialogDetail = "Exceed limitation for number of Technician account (4 account)";
                    _clearTextField = false;
                } else if (_codeCreateAccountCheck === 6) {
                    messNewUserDialogContent = "Cannot create account";
                    messNewUserDialogDetail = "Exceed limitation for number of Operator account (10 account)";
                    _clearTextField = false;
                } else {
                    messNewUserDialogContent = "Exception Occurred";
                    messNewUserDialogDetail = "An unexpected exception occurred while processing your request.";
                    _clearTextField = true;
                }
            }

            var component = Qt.createComponent("../Dialog/ModalDialogBox.qml");
            if (component.status === Component.Ready) {
            var messNewUserDialog = component.createObject(mainWindow,{_typeDialog:-1,
                                                            _content:messNewUserDialogContent,
                                                            _detailContent:messNewUserDialogDetail,
                                                            _isFunctional: false
                                                          });
            messNewUserDialog.open();
            // Do after open
            if (thisItem._clearTextField){
                stackviewLogin.createUserClear = ""
                stackviewLogin.createPassClear = ""
                stackviewLogin.createPassconfirmClear = ""
            }
            else{
                stackviewLogin.createUserClear = stackviewLogin.createUserClear
                stackviewLogin.createPassClear = stackviewLogin.createPassClear
                stackviewLogin.createPassconfirmClear = stackviewLogin.createPassconfirmClear
            }
            }
        }

        function checkUserLogin(userT, passwordT, rememberT) {
            _clearTextField = false;
            var messLoginDialogContent=""
            var messLoginDialogDetail=""

            if (_username !== "" || _level !== "") {
                messLoginDialogContent = "Please Log Out";
                messLoginDialogDetail = "Please log out of the current account before attempting to log in again.";
                _clearTextField = false;
            } else if (userT === "") {
                messLoginDialogContent = "Username Required";
                messLoginDialogDetail = "Username cannot be empty. Please enter your username.";
                _clearTextField = false;
            } else if (passwordT === "") {
                messLoginDialogContent = "Password Required";
                messLoginDialogDetail = "Please enter your password to proceed.";
                _clearTextField = false;
            } else {
                loginVM.checkUserLoginInDatabase(userT, passwordT, rememberT);
                if (_codeLoginCheck === 0) {
                    messLoginDialogContent = "Login Successful";
                    messLoginDialogDetail = "You have successfully logged in.";
                    _clearTextField = true;
                    stackviewLogin.loginUserClear = ""
                    stackviewLogin.loginPassClear = ""
					popupLogin.close();

                    var boolValue = (thisItem._level !== "") && (thisItem._username !== "");
                    var isAdmin = (thisItem._username === "Admin") && productionVM.enabledForRunLot ;
                    var enableList = [!boolValue, isAdmin, boolValue, isAdmin, boolValue];
                    internalLoginFunction.changeEnableSelectMode(enableList);
                    return
                } else if (_codeLoginCheck === 1) {
                    messLoginDialogContent = "Incorrect Password";
                    messLoginDialogDetail = "The entered password does not match our records. Please check your password and try again.";
                    _clearTextField = false;
                } else if (_codeLoginCheck === 2) {
                    messLoginDialogContent = "User Not Found";
                    messLoginDialogDetail = "The entered username is not registered in our system. Please check your username and try again.";
                    _clearTextField = false;
                } else {
                    messLoginDialogContent = "Exception Occurred";
                    messLoginDialogDetail = "An unexpected exception occurred while processing your request.";
                    _clearTextField = true;
                }
            }
            if (_clearTextField){
                stackviewLogin.loginUserClear = ""
                stackviewLogin.loginPassClear = ""
            }
            var component = Qt.createComponent("../Dialog/ModalDialogBox.qml");
            if (component.status === Component.Ready) {
                var messLoginDialog = component.createObject(mainWindow,{_typeDialog:-1,
                                                                _content:messLoginDialogContent,
                                                                _detailContent:messLoginDialogDetail,
                                                                _isFunctional: false
                                                              });

                messLoginDialog.open()
            }
        }


        function checkPasswordChange(passwordT, passwordconfirmT) {
            _clearTextField = false;
            var messChangePasswordDialogContent=""
            var messChangePasswordDialogDetail=""

            if (_username === "" || _level === "") {
                messChangePasswordDialogContent = "Account Login Required";
                messChangePasswordDialogDetail = "Please log in to your account before attempting to change your password.";
                _clearTextField = true;
            } else if (passwordT === "" || passwordconfirmT === "") {
                messChangePasswordDialogContent = "Password Required";
                messChangePasswordDialogDetail = "Both password and password confirmation fields are required.";
                _clearTextField = false;
            } else if (passwordT !== passwordconfirmT) {
                messChangePasswordDialogContent = "Passwords Do Not Match";
                messChangePasswordDialogDetail = "The password you entered does not match the confirmed password.";
                _clearTextField = false;
            } else {
                loginVM.checkPasswordChangeInDatabase(passwordT);
                if (_codeChangePasswordCheck === 0) {
                    messChangePasswordDialogContent = "Password Changed Successfully";
                    messChangePasswordDialogDetail = "Your password has been successfully changed.";
                    _clearTextField = true;
                } else if (_codeChangePasswordCheck === 1) {
                    messChangePasswordDialogContent = "Password Unchanged";
                    messChangePasswordDialogDetail = "The new password is the same as the previous one. Please enter a different password.";
                    _clearTextField = false;
                }
                else if (_codeChangePasswordCheck === 2) {
                    messChangePasswordDialogContent = "Invalid Password";
                    messChangePasswordDialogDetail = "Passwords must have at least 8 characters and contain both uppercase and lowercase letters.";
                    _clearTextField = false;
                }
                else {
                    messChangePasswordDialogContent = "Exception Occurred";
                    messChangePasswordDialogDetail = "An unexpected exception occurred while processing your request.";
                    _clearTextField = true;
                }
            }

            var component = Qt.createComponent("../Dialog/ModalDialogBox.qml");
            if (component.status === Component.Ready) {
            var messChangePasswordDialog = component.createObject(mainWindow,{_typeDialog:-1,
                                                                _content:messChangePasswordDialogContent,
                                                                _detailContent:messChangePasswordDialogDetail,
                                                                _isFunctional: false
                                                          });

                messChangePasswordDialog.open();
                if (thisItem._clearTextField){
                    stackviewLogin.changePassClear = ""
                    stackviewLogin.changePassConfirmClear = ""
                }
                else{
                    stackviewLogin.changePassClear  = stackviewLogin.changePassClear
                    stackviewLogin.changePassConfirmClear = stackviewLogin.changePassConfirmClear
                }
            }
        }

        function checkDeleteAccount(userT, levelString) {
            var messDeleteAccountDialogContent=""
            var messDeleteAccountDialogDetail=""

            if (_username === "" || _level === "") {
                messDeleteAccountDialogContent = "Account Login Required";
                messDeleteAccountDialogDetail = "Please log in to your account before attempting to delete it.";
            } else {
                loginVM.checkDeleteAccountInDatabase();
                if (_codeDeleteAccountCheck === 0) {
                    messDeleteAccountDialogContent = "Account Deleted Successfully";
                    messDeleteAccountDialogDetail = "Your account has been successfully deleted.";
                } else if (_codeDeleteAccountCheck === 1) {
                    messDeleteAccountDialogContent = "Deletion Failed";
                    messDeleteAccountDialogDetail = "Failed to delete the account. The account you are trying to delete has a higher access level than your current account.";
                } else if (_codeDeleteAccountCheck === 2) {
                    messDeleteAccountDialogContent = "No Account Selected for Deletion";
                    messDeleteAccountDialogDetail = "Please select the account you wish to delete.";
                } else {
                    messDeleteAccountDialogContent = "Exception Occurred";
                    messDeleteAccountDialogDetail = "An unexpected exception occurred while processing your request.";
                }
            }

            var component = Qt.createComponent("../Dialog/ModalDialogBox.qml");
            if (component.status === Component.Ready) {
                var messDeleteAccountDialog = component.createObject(mainWindow,{_typeDialog:-1,
                                                              _content:messDeleteAccountDialogContent,
                                                              _detailContent:messDeleteAccountDialogDetail,
                                                              _isFunctional: false
                                                              });


            messDeleteAccountDialog.open();
            }
        }

        function checkUserLogout() {
            if (_username === "" || _level === "") {
                thisItem._codeLogoutCheck = 1;

                var component = Qt.createComponent("../Dialog/ModalDialogBox.qml");
                if (component.status === Component.Ready) {
                    var messLogoutAccountDialog = component.createObject(mainWindow,{_typeDialog:-1,
                                                                    _content:"Account Not Logged In",
                                                                    _detailContent:"You need to log in before you can log out.",
                                                                    _isFunctional: false
                                                                  });

                    messLogoutAccountDialog.open();

                    _clearTextAfterLogout = true
                }
            } else {
                thisItem._codeLogoutCheck = 0;

                loginVM.clearDataCurrentAccount();
                internalLoginFunction.clearDataCurrentAccount();

                var boolValue = (thisItem._level !== "") && (thisItem._username !== "");
                var enableList = [!boolValue, boolValue && productionVM.enabledForRunLot, boolValue, boolValue && productionVM.enabledForRunLot, boolValue]; // login, create, change, delete, logout
                internalLoginFunction.changeEnableSelectMode(enableList);
                stackviewLogin.loginUserClear = ""
                stackviewLogin.loginPassClear = ""
            }
        }

        function clearDataCurrentAccount(){
            thisItem._username = ""
            thisItem._level = ""
           _clearTextAfterLogout = true
        }

        function createFakePassword(lenghtT){
            var temp ="";
            for (var i = 0; i < lenghtT; i ++)
                temp += "*"
            return temp
        }
    }
    /* MessageDialog */
    ListModel{
        id:listModelFunction
    }
    Component.onCompleted: {
        dashboardVM.isCovered = true
        loginVM.loadListAccount()
        stackviewLogin.push(loginPanel)
        listModelFunction.append({"name":"Login",
            "color":color_function_item,
            "family":"Adobe Gothic Std B",
            "fontsize":_fontsize_funtion_view,
			enableSelect: true
        })
        listModelFunction.append({"name":"Create New Account",
            "color":color_function_item,
            "family":"Adobe Gothic Std B",
            "fontsize":_fontsize_funtion_view,
			enableSelect: false
        })
        listModelFunction.append({"name":"Change Password",
            "color":color_function_item,
            "family":"Adobe Gothic Std B",
            "fontsize":_fontsize_funtion_view,
			enableSelect: false
        })
        listModelFunction.append({"name":"Delete User Account",
            "color":color_function_item,
            "family":"Adobe Gothic Std B",
            "fontsize":_fontsize_funtion_view,
			enableSelect: false
        })
        listModelFunction.append({"name":"Logout",
            "color":color_function_item,
            "family":"Adobe Gothic Std B",
            "fontsize":_fontsize_funtion_view,
			enableSelect: false
        })
    }

    function requestShowPanel(_nameFunction){

        switch (_nameFunction) {
            case "Login":
                stackviewLogin.clear()
                stackviewLogin.push(loginPanel)
                break;
            case "Create New Account":
                stackviewLogin.clear()
                stackviewLogin.push(newUserPanel)
                break;
            case "Change Password":
                stackviewLogin.clear()
                stackviewLogin.push(newPasswordPanel)
                break;
            case "Delete User Account":
                stackviewLogin.clear()
                stackviewLogin.push(deleteUserAccountPanel)
                break;
            case "Logout":
                internalLoginFunction.checkUserLogout()
                if (thisItem._codeLogoutCheck == 0){
                    stackviewLogin.clear()
                    stackviewLogin.push(loginPanel)
                }
                else{
                    break;
                }
                break;
            default:
                stackviewLogin.clear()
                stackviewLogin.push(loginPanel)
        }
    }
    function setTextStyle(parentElement){
        for (var i = 0; i < parentElement.children.length; ++i) {
            if (parentElement.children[i].useStyle) {  // apply style for Text(control) or not ?
                parentElement.children[i].color = "#ffdf00";
                parentElement.children[i].font.pointSize =_fontsize_inforLogin_view;
                parentElement.children[i].font.family = "Adobe Gothic Std B" ;
                parentElement.children[i].font.bold = true;
                parentElement.children[i].verticalAlignment=Text.AlignVCenter;
                parentElement.children[i].horizontalAlignment=Text.AlignHCenter;
            }
        }
    }

    Component{  /* Component Header*/
        id:header_view
        Rectangle{
            anchors.fill: parent
            color: "transparent"
            Image{
                anchors.centerIn: parent
                id:image
                width:parent.width*0.2
                height:width
                source: "../../Resources/ED logo.png"
                fillMode:Image.PreserveAspectFit
                anchors.horizontalCenter: parent.horizontalCenter
            }
        }
    }

    Component{  /* Component Infor Login*/
        id:inforLogin_view
        Row{
            id: login
            anchors.fill: parent
            Component.onCompleted: {
                thisItem.setTextStyle(col_infor_1)
                thisItem.setTextStyle(col_infor_2)
            }
            Column{
                id:col_infor_1
                padding: 0
                spacing: 0
                width: parent.width*0.45
                Text{
                    id: usernameText
                    text:"Username"
                    property bool useStyle:true
                    leftPadding: 25
                    bottomPadding: 10
                    height: implicitHeight
                }
                Text{
                    id: accesslevelText
                    text:"Access Level"
                    property bool useStyle:true
                    leftPadding: 25
                    topPadding: 15
                    height: implicitHeight
                }
            }
            Rectangle{
                width: parent.width*0.1
                height: parent.height
                color:"transparent"
                border.color: "transparent"
                Rectangle{
                    width: parent.height * 0.7
                    height: 1
                    anchors.top: parent.top
                    anchors.horizontalCenter: parent.horizontalCenter
                    anchors.topMargin: width * 0.4
                    color: "ghostwhite"
                    opacity: 0.5
                    rotation: 90
                }
            }
            Column{
                id:col_infor_2
                padding: 0
                spacing: 0
                width: parent.width*0.45
                Text{
                    id: userLoginText
                    text:thisItem._username
                    property bool useStyle:true
                    leftPadding: 10
                    bottomPadding: 10
                    height: implicitHeight
                }
                Text{
                    id: levelLoginText
                    text:thisItem._level
                    property bool useStyle:true
                    leftPadding: 10
                    topPadding: 15
                    height: implicitHeight
                }
            }
        }
    }

    Component{  /* Component Function Login*/
        id:funtion_view
        ListView{
            clip:false
            interactive:true
            model: listModelFunction
            spacing:20
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.verticalCenter: parent.verticalCenter
            delegate:Rectangle{
                id: rect_function_view
                enabled: stackviewLogin.enable_mouseAction_function_view && enableSelect
				opacity: enabled ? 1.0 : 0.2
                color:"#222222"
                height:txt_fun.height * 2.5
                radius:5
                width:parent.width *0.95
                scale:  mouseArea.containsMouse ? 1 : 0.9
                smooth: mouseArea.containsMouse
                anchors{
                    left:parent.left
                    leftMargin:10
                    right:parent.right
                    rightMargin:10
                }
                MouseArea{
                    hoverEnabled: true
                    id:mouseArea
                    acceptedButtons: Qt.LeftButton | Qt.RightButton
                    onEntered: {
                        txt_fun.color = "lime"
                    }
                    onExited:  {
                        txt_fun.color = model.color
                    }
                    onClicked:{
                        loginVM.updateIdleTimer()
                        if(mouse.button === Qt.LeftButton){
                            thisItem.requestShowPanel(model.name)
                        }
                    }
                    anchors.fill: parent
                }
                Text{
                    id:txt_fun
                    anchors{
                        left:parent.left
                        leftMargin:10
                        right:parent.right
                        rightMargin:10
                    }
                    height:15
                    font.pointSize:_fontsize_funtion_view
                    text:model.name
                    color:model.color
                    font.family:model.family
                    anchors.verticalCenter: parent.verticalCenter
                    verticalAlignment:Text.AlignVCenter
                }
            }
        }
    }

    Component{  /* Login Panel*/
        id: loginPanel
        Item{
            //logged in view
            ColumnLayout{
                visible: thisItem._level !== ""
                anchors.centerIn: parent
                spacing: 30
                MaterialDesignIcon{
                    name: "check"
                    color: "green"
                    size: 150
                    Layout.alignment :Qt.AlignHCenter
                    Layout.preferredWidth: 150
                    Layout.preferredHeight: 150
                }
                Text {
                    id: name
                    Layout.alignment :Qt.AlignHCenter
                    font.pointSize: 17
                    color: "white"
                    text: qsTr("You are already logged in.")
                }
            }
            Column{
                //log in view
                visible: thisItem._level === ""
                height:parent.height
                width: parent.width*0.8
                anchors{
                    horizontalCenter: parent.horizontalCenter
                }
                spacing: 0
                SpaceV{_space:94}
                Text{
                    height:contentHeight
                    width: contentWidth
                    text:"Login"
                    font.pointSize:18
                    color:"deepskyblue"
                    font.bold:true
                    font.family:"Adobe Gothic Std B"
                    verticalAlignment:Text.AlignVCenter
                    horizontalAlignment:Text.AlignHCenter

                }
                SpaceV{_space:39}
                Column{
                    height:parent.height*0.3
                    width:parent.width
                    Rectangle{
                        width: parent.width*0.9
                        height: 40
                        color: "#505557"
                        border{
                            width: 2
                            color: "white"
                        }
                        radius: 5
                        FieldTextWithKeyboard{
                            id: txt_username
                            _yTextBoard: 100
                            placeholderText: qsTr("Enter user name")
                            text: stackviewLogin.loginUserClear
                            anchors.fill: parent
                            anchors.centerIn: parent
                            background: null
                            horizontalAlignment: Text.AlignLeft
                            verticalAlignment: Text.AlignVCenter
                            leftPadding: 20
                            font.pointSize: 14
                            maximumLength: 20
    						validator: RegExpValidator { regExp: /^[a-zA-Z0-9]*$/ }
                            onEnterKeyPress:{
                                loginVM.updateIdleTimer()
                                stackviewLogin.loginUserClear = txt_username.text
                                stackviewLogin.loginPassClear = txt_password.text
                                internalLoginFunction.checkUserLogin(txt_username.text, txt_password.text, stackviewLogin.remember_account)
                            }

                            Connections{
                                target: thisItem
                                ignoreUnknownSignals: true
                                onViewKeyboardOnInit:{
                                    txt_username.forceActiveFocus()
                                }
                            }
                        }
                    }
                    SpaceV{_space: 10}
                    Rectangle{
                        width: parent.width*0.9
                        height: 40
                        color: "#505557"
                        border{
                            width: 2
                            color: "white"
                        }
                        radius: 5
                        FieldTextWithKeyboard{
                            id: txt_password
                            _yTextBoard: 100
                            placeholderText: qsTr("Enter password")
                            text: stackviewLogin.loginPassClear
                            echoMode: TextInput.Password
                            property int countFirstText: 0
                            anchors.fill: parent
                            anchors.centerIn: parent
                            background: null
                            horizontalAlignment: Text.AlignLeft
                            verticalAlignment: Text.AlignVCenter
                            leftPadding: 20
                            font.pointSize: 14
                            maximumLength: 20
    						validator: RegExpValidator { regExp: /^[a-zA-Z0-9]*$/ }
                            onTextChanged: {
                                if(countFirstText === 0) {
                                    countFirstText = countFirstText + 1
                                    return
                                }
                                if(countFirstText > 0 ) {
                                    loginVM.setflagOvercheckPassword(false)
                                }
                                countFirstText = countFirstText + 1
                            }
                            onEnterKeyPress:{
                                loginVM.updateIdleTimer()
                                stackviewLogin.loginUserClear = txt_username.text
                                stackviewLogin.loginPassClear = txt_password.text
                                internalLoginFunction.checkUserLogin(txt_username.text, txt_password.text, stackviewLogin.remember_account)
                            }
                        }
                        Rectangle{
                            anchors.right: txt_password.right
                            anchors.rightMargin: 5
                            color: "transparent"
                            height: parent.height
                            width: height
                            Image {
                                anchors.fill: parent
                                property bool passwordVisible: false
                                source: passwordVisible ? "../../Resources/eye_open_black.png" : "../../Resources/eye_close_black.png"
                                MouseArea{
                                    anchors.fill: parent
                                    onClicked: {
                                        loginVM.updateIdleTimer()
                                        parent.passwordVisible = ! parent.passwordVisible
                                        if (parent.passwordVisible) txt_password.echoMode = TextInput.Normal
                                        else txt_password.echoMode = TextInput.Password
                                    }
                                }
                            }
                        }
                    }
                }
                SpaceV{_space:-90}

                SpaceV{_space:5}
                Row{
                    height:parent.height*0.2-10
                    width:parent.width
                    anchors{
                        left:parent.left
                    }
                    ButtonText{
                        width:parent.width*0.4
                        height:35
                        _text:"Login"
                        _btnColorMouseOver:"deepskyblue"
                        focusPolicy: Qt.ClickFocus
                        focusReason: Qt.OtherFocusReason
                        onClicked:{
							loginVM.updateIdleTimer()
							stackviewLogin.loginUserClear = txt_username.text
							stackviewLogin.loginPassClear = txt_password.text
							internalLoginFunction.checkUserLogin(txt_username.text, txt_password.text, stackviewLogin.remember_account)
						}
                    }
                    SpaceH{_space:parent.width*0.1}
                    ButtonText{
                        width:parent.width*0.4
                        height:35
                        _text:"Close"
                        _btnColorMouseOver:"deepskyblue"
                        MouseArea{
                            enabled: true
                            acceptedButtons: Qt.LeftButton | Qt.RightButton
                            anchors.fill: parent
                            onClicked:{
                                loginVM.updateIdleTimer()
                                if (mouse.button === Qt.LeftButton){
									thisItem.quitSignal()
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    Component{  /* New User Panel*/
        id:newUserPanel
        Rectangle{
            color:"transparent"
            Column{
                height:parent.height
                width: parent.width*0.8
                anchors{
                    leftMargin:10
                    rightMargin:parent.width*0.1
                    horizontalCenter: parent.horizontalCenter
                }
                SpaceV{_space:94}
                Text{
                    height:contentHeight
                    width: contentWidth
                    text:"Create New Account"
                    color:"deepskyblue"
                    font.pointSize:18
                    font.bold:true
                    horizontalAlignment:Text.AlignHCenter
                    verticalAlignment:Text.AlignVCenter
                }
                SpaceV{_space:39}
                Rectangle{
                    width: parent.width*0.9
                    height: 40
                    color: "#505557"
                    border{
                        width: 2
                        color: "white"
                    }
                    radius: 5
                    FieldTextWithKeyboard{
                        id:txt_username_new
                        _yTextBoard: 100
                        placeholderText: qsTr("Enter new user name")
                        text: stackviewLogin.createUserClear
                        anchors.fill: parent
                        anchors.centerIn: parent
                        background: null
                        horizontalAlignment: Text.AlignLeft
                        verticalAlignment: Text.AlignVCenter
                        leftPadding: 20
                        font.pointSize: 14
                        maximumLength: 20
						validator: RegExpValidator { regExp: /^[a-zA-Z0-9]*$/ }
                        onEnterKeyPress:{
                            loginVM.updateIdleTimer()
                            stackviewLogin.createUserClear = txt_username_new.text
                            stackviewLogin.createPassClear = txt_password_new.text
                            stackviewLogin.createPassconfirmClear = txt_password_new_confirm.text
                            var levelBool = adminBtnIsOf || technicianIsOf || operatorIsOf
                            var levelString = adminBtnIsOf ? "Administrator" : (technicianIsOf ? "Technician" : (operatorIsOf ? "Operator" :""))
                            internalLoginFunction.checkAccountCreate(txt_username_new.text, txt_password_new.text,
                                                                        txt_password_new_confirm.text,
                                                                        levelBool, levelString)
                        }
                    }
                }
                SpaceV{_space:10}
                Rectangle{
                    height: 40
                    width:parent.width
                    color: "transparent"
                    Rectangle{
                        width: parent.width*0.9
                        height: 40
                        color: "#505557"
                        border{
                            width: 2
                            color: "white"
                        }
                        radius: 5
                        FieldTextWithKeyboard{
                            id: txt_password_new
                            _yTextBoard: 100
                            placeholderText: qsTr("Enter new password")
                            text:stackviewLogin.createPassClear
                            anchors.fill: parent
                            anchors.centerIn: parent
                            background: null
                            horizontalAlignment: Text.AlignLeft
                            verticalAlignment: Text.AlignVCenter
                            leftPadding: 20
                            echoMode: TextInput.Password
                            font.pointSize: 14
                            maximumLength: 20
    						validator: RegExpValidator { regExp: /^[a-zA-Z0-9]*$/ }
                            onEnterKeyPress:{
                                loginVM.updateIdleTimer()
                                stackviewLogin.createUserClear = txt_username_new.text
                                stackviewLogin.createPassClear = txt_password_new.text
                                stackviewLogin.createPassconfirmClear = txt_password_new_confirm.text
                                var levelBool = adminBtnIsOf || technicianIsOf || operatorIsOf
                                var levelString = adminBtnIsOf ? "Administrator" : (technicianIsOf ? "Technician" : (operatorIsOf ? "Operator" :""))
                                internalLoginFunction.checkAccountCreate(txt_username_new.text, txt_password_new.text,
                                                                            txt_password_new_confirm.text,
                                                                            levelBool, levelString)
                            }
                        }
                    }
                    Rectangle{
                        anchors.right: txt_password_new.parent.right
                        anchors.rightMargin: 5
                        color: "transparent"
                        width: parent.width*0.1
                        height: parent.height
                        Image {
                            anchors.fill: parent
                            property bool passwordVisible: false
                            source: passwordVisible ? "../../Resources/eye_open_black.png" : "../../Resources/eye_close_black.png"
                            MouseArea{
                                anchors.fill: parent
                                onClicked: {
                                    loginVM.updateIdleTimer()
                                    parent.passwordVisible = ! parent.passwordVisible
                                    if (parent.passwordVisible) txt_password_new.echoMode = TextInput.Normal
                                    else txt_password_new.echoMode = TextInput.Password
                                }
                            }
                        }
                    }
                }
                SpaceV{_space:10}
                Rectangle{
                    height: 40
                    width:parent.width
                    color: "transparent"
                    Rectangle{
                        width: parent.width*0.9
                        height: 40
                        color: "#505557"
                        border{
                            width: 2
                            color: "white"
                        }
                        radius: 5
                        FieldTextWithKeyboard{
                            id: txt_password_new_confirm
                            _yTextBoard: 100
                            placeholderText: qsTr("Confirm new password")
                            text: stackviewLogin.createPassconfirmClear
                            echoMode: TextInput.Password
                            anchors.fill: parent
                            anchors.centerIn: parent
                            background: null
                            horizontalAlignment: Text.AlignLeft
                            verticalAlignment: Text.AlignVCenter
                            leftPadding: 20
                            font.pointSize: 14
                            maximumLength: 20
    						validator: RegExpValidator { regExp: /^[a-zA-Z0-9]*$/ }
                            onEnterKeyPress:{
                                loginVM.updateIdleTimer()
                                stackviewLogin.createUserClear = txt_username_new.text
                                stackviewLogin.createPassClear = txt_password_new.text
                                stackviewLogin.createPassconfirmClear = txt_password_new_confirm.text
                                var levelBool = adminBtnIsOf || technicianIsOf || operatorIsOf
                                var levelString = adminBtnIsOf ? "Administrator" : (technicianIsOf ? "Technician" : (operatorIsOf ? "Operator" :""))
                                internalLoginFunction.checkAccountCreate(txt_username_new.text, txt_password_new.text,
                                                                            txt_password_new_confirm.text,
                                                                            levelBool, levelString)
                            }
                        }
                    }
                    Rectangle{
                        anchors.right: txt_password_new_confirm.parent.right
                        anchors.rightMargin: 5
                        color: "transparent"
                        width: parent.width*0.1
                        height: parent.height

                        Image {
                            anchors.fill: parent
                            property bool passwordVisible: false
                            source: passwordVisible ? "../../Resources/eye_open_black.png" : "../../Resources/eye_close_black.png"
                            MouseArea{
                                anchors.fill: parent
                                onClicked: {
                                    loginVM.updateIdleTimer()
                                    parent.passwordVisible = ! parent.passwordVisible
                                    if (parent.passwordVisible) txt_password_new_confirm.echoMode = TextInput.Normal
                                    else txt_password_new_confirm.echoMode = TextInput.Password
                                }
                            }
                        }
                    }
                }
                SpaceV{_space:30}
                Row{
                    height:parent.height*0.125
                    width:parent.width
                    anchors{
                        left:parent.left
                        leftMargin:0
                        right:parent.right
                        rightMargin:parent.width *0.1
                    }
                    SpaceH{_space: - 30}

                    Rectangle {
                        id: adminBtn
                        height:50
                        width:parent.width*0.35
                        color: adminBtnIsOf? "green":"gray"
                        Text{
                            anchors.centerIn: parent
                            text: qsTr("Admin")
                            font.pixelSize: 20
                        }
                        MouseArea{
                            anchors.fill: parent
                            onClicked: {
                                loginVM.updateIdleTimer()
                                adminBtnIsOf = !adminBtnIsOf
                                if(adminBtnIsOf){
                                    technicianIsOf = false
                                    operatorIsOf = false
                                }
                            }
                        }
                    }

                    SpaceH{_space:Math.round(parent.width*0.05)}
                    Rectangle {
                        id: technicianBtn
                        height:50
                        width:parent.width*0.35
                        color: technicianIsOf? "green":"gray"
                        Text{
                            anchors.centerIn: parent
                            font.pixelSize: 20
                        text: qsTr("Technician")
                        }
                        MouseArea{
                            anchors.fill: parent
                        onClicked: {
                            loginVM.updateIdleTimer()
                            technicianIsOf = !technicianIsOf
                            if(technicianIsOf){
                                adminBtnIsOf = false
                                operatorIsOf = false
                            }
                        }
                        }
                    }
                    SpaceH{_space:Math.round(parent.width*0.05)}
                    Rectangle {
                        id: techBtn
                        height:50
                        width:parent.width*0.35
                        color: operatorIsOf? "green":"gray"
                        Text{
                            anchors.centerIn: parent
                        text: qsTr("Operator")
                            font.pixelSize: 20
                        }

                        MouseArea{
                            anchors.fill: parent
                        onClicked: {
                            loginVM.updateIdleTimer()
                            operatorIsOf = !operatorIsOf
                            if(operatorIsOf){
                                adminBtnIsOf = false
                                technicianIsOf = false
                            }
                        }
                        }
                    }
                }
                SpaceV{_space:-10}
                Row{
                    height:parent.height*0.2 - 10
                    width:parent.width
                    anchors.horizontalCenter: parent.horizontalCenter

                    ButtonText{
                        width:parent.width*0.4
                        height:35
                        _text:"Create"
                        _btnColorMouseOver:"deepskyblue"
                        MouseArea{
                            enabled: true
                            acceptedButtons: Qt.LeftButton | Qt.RightButton
                            anchors.fill: parent
                            onClicked:{
                                loginVM.updateIdleTimer()
                                if (mouse.button === Qt.LeftButton){ // Decorate mouse down event
                                    stackviewLogin.createUserClear = txt_username_new.text
                                    stackviewLogin.createPassClear = txt_password_new.text
                                    stackviewLogin.createPassconfirmClear = txt_password_new_confirm.text
                                    var levelBool = adminBtnIsOf || technicianIsOf || operatorIsOf
                                    var levelString = adminBtnIsOf ? "Administrator" : (technicianIsOf ? "Technician" : (operatorIsOf ? "Operator" :""))
                                    internalLoginFunction.checkAccountCreate(txt_username_new.text, txt_password_new.text,
                                                                             txt_password_new_confirm.text,
                                                                             levelBool, levelString)

                                }
                            }
                        }
                    }
                    SpaceH{_space:parent.width*0.1}
                    ButtonText{
                        width:parent.width*0.4
                        height:35
                        _text:"Clear"
                        _btnColorMouseOver:"deepskyblue"
                        onClicked:{
                            loginVM.updateIdleTimer()
                                stackviewLogin.createUserClear = ""
                                stackviewLogin.createPassClear = ""
                                stackviewLogin.createPassconfirmClear = ""
                                txt_username_new.text = ""
                                txt_password_new.text = ""
                                txt_password_new_confirm.text = ""
                                adminBtnIsOf = false
                                technicianIsOf = false
                                operatorIsOf = false
                        }
                    }
                }
            }
        }
    }

    Component{  /* New Password Panel*/
        id:newPasswordPanel
        Rectangle{
            color:"transparent"
            Column{
                height:parent.height
                width: parent.width*0.8
                anchors{
                    leftMargin:10
                    rightMargin:parent.width*0.1
                    horizontalCenter: parent.horizontalCenter
                }
                SpaceV{_space:94}
                Text{
                    height:contentHeight
                    width: contentWidth
                    text:"Change Password"
                    color:"deepskyblue"
                    font.pointSize:18
                    font.bold:true
                    horizontalAlignment:Text.AlignHCenter
                    verticalAlignment:Text.AlignVCenter
                }
                SpaceV{_space:15}
                Rectangle{
                    height: 40
                    width:parent.width
                    color: "transparent"
                    Rectangle{
                        width: parent.width*0.9
                        height: 40
                        color: "#505557"
                        border{
                            width: 2
                            color: "white"
                        }
                        radius: 5
                        FieldTextWithKeyboard{
                            id:txt_password_old
                            _yTextBoard: 100
                            placeholderText: qsTr("Enter current password")
                            text: stackviewLogin.changePassClear
                            echoMode: TextInput.Password
                            anchors.fill: parent
                            anchors.centerIn: parent
                            background: null
                            horizontalAlignment: Text.AlignLeft
                            verticalAlignment: Text.AlignVCenter
                            leftPadding: 20
                            font.pointSize: 14
                            maximumLength: 20
    						validator: RegExpValidator { regExp: /^[a-zA-Z0-9]*$/ }
                            onEnterKeyPress:{
                                loginVM.updateIdleTimer()
                                if(txt_password_old.text === thisItem._currentPw )
                                {
                                    stackviewLogin.changePassClear = txt_password_new.text
                                    stackviewLogin.changePassConfirmClear = txt_password_new_confirm.text
                                    internalLoginFunction.checkPasswordChange(txt_password_new.text, txt_password_new_confirm.text)

                                }
                                else
                                {
                                    _clearTextField = false;

                                    var component = Qt.createComponent("../Dialog/ModalDialogBox.qml");
                                    if (component.status === Component.Ready) {
                                        var messChangePasswordDialog = component.createObject(mainWindow,{_typeDialog:-1,
                                                                                        _content:"Current password incorrect",
                                                                                        _detailContent:"",
                                                                                        _isFunctional: false
                                                                                    });
                                        //messChangePasswordDialog.sendNotify()
                                        messChangePasswordDialog.open();
                                        if (thisItem._clearTextField){
                                            stackviewLogin.changePassClear = ""
                                            stackviewLogin.changePassConfirmClear = ""
                                        }
                                        else{
                                            stackviewLogin.changePassClear  = stackviewLogin.changePassClear
                                            stackviewLogin.changePassConfirmClear = stackviewLogin.changePassConfirmClear
                                        }
                                    }
                                }
                            }
                        }
                    }
                    Rectangle{
                        anchors.right: txt_password_old.parent.right
                        anchors.rightMargin: 5
                        color: "transparent"
                        width: parent.width*0.1
                        height: parent.height
                        Image {
                            anchors.fill: parent
                            property bool passwordVisible: false
                            source: passwordVisible ? "../../Resources/eye_open_black.png" : "../../Resources/eye_close_black.png"
                            MouseArea{
                                anchors.fill: parent
                                onClicked: {
                                    loginVM.updateIdleTimer()
                                    parent.passwordVisible = ! parent.passwordVisible
                                    if (parent.passwordVisible) txt_password_old.echoMode = TextInput.Normal
                                    else txt_password_old.echoMode = TextInput.Password
                                }
                            }
                        }
                    }
                }

                SpaceV{_space:10}
                Rectangle{
                    height: 40
                    width:parent.width
                    color: "transparent"
                    Rectangle{
                        width: parent.width*0.9
                        height: 40
                        color: "#505557"
                        border{
                            width: 2
                            color: "white"
                        }
                        radius: 5
                        FieldTextWithKeyboard{
                            id:txt_password_new
                            _yTextBoard: 100
                            text:stackviewLogin.changePassClear
                            placeholderText: qsTr("Enter new Password")
                            echoMode: TextInput.Password
                            anchors.fill: parent
                            anchors.centerIn: parent
                            background: null
                            horizontalAlignment: Text.AlignLeft
                            verticalAlignment: Text.AlignVCenter
                            leftPadding: 20
                            font.pointSize: 14
                            maximumLength: 20
    						validator: RegExpValidator { regExp: /^[a-zA-Z0-9]*$/ }
                            onEnterKeyPress:{
                                loginVM.updateIdleTimer()
                                if(txt_password_old.text === thisItem._currentPw )
                                {
                                    stackviewLogin.changePassClear = txt_password_new.text
                                    stackviewLogin.changePassConfirmClear = txt_password_new_confirm.text
                                    internalLoginFunction.checkPasswordChange(txt_password_new.text, txt_password_new_confirm.text)

                                }
                                else
                                {
                                    _clearTextField = false;

                                    var component = Qt.createComponent("../Dialog/ModalDialogBox.qml");
                                    if (component.status === Component.Ready) {
                                        var messChangePasswordDialog = component.createObject(mainWindow,{_typeDialog:-1,
                                                                                        _content:"Current password incorrect",
                                                                                        _detailContent:"",
                                                                                        _isFunctional: false
                                                                                    });
                                        //messChangePasswordDialog.sendNotify()
                                        messChangePasswordDialog.open();
                                        if (thisItem._clearTextField){
                                            stackviewLogin.changePassClear = ""
                                            stackviewLogin.changePassConfirmClear = ""
                                        }
                                        else{
                                            stackviewLogin.changePassClear  = stackviewLogin.changePassClear
                                            stackviewLogin.changePassConfirmClear = stackviewLogin.changePassConfirmClear
                                        }
                                    }
                                }
                            }
                        }
                    }
                    Rectangle{
                        anchors.right: txt_password_new.parent.right
                        anchors.rightMargin: 5
                        color: "transparent"
                        width: parent.width*0.1
                        height: parent.height
                        Image {
                            anchors.fill: parent
                            property bool passwordVisible: false
                            source: passwordVisible ? "../../Resources/eye_open_black.png" : "../../Resources/eye_close_black.png"
                            MouseArea{
                                anchors.fill: parent
                                onClicked: {
                                    loginVM.updateIdleTimer()
                                    parent.passwordVisible = ! parent.passwordVisible
                                    if (parent.passwordVisible) txt_password_new.echoMode = TextInput.Normal
                                    else txt_password_new.echoMode = TextInput.Password
                                }
                            }
                        }
                    }
                }
                SpaceV{_space:10}
                Rectangle{
                    height: 40
                    width:parent.width
                    color: "transparent"
                    Rectangle{
                        width: parent.width*0.9
                        height: 40
                        color: "#505557"
                        border{
                            width: 2
                            color: "white"
                        }
                        radius: 5
                        FieldTextWithKeyboard{
                            id: txt_password_new_confirm
                            _yTextBoard: 100
                            text: stackviewLogin.changePassConfirmClear
                            placeholderText: qsTr("Confirm new password")
                            echoMode: TextInput.Password
                            anchors.fill: parent
                            anchors.centerIn: parent
                            background: null
                            horizontalAlignment: Text.AlignLeft
                            verticalAlignment: Text.AlignVCenter
                            leftPadding: 20
                            font.pointSize: 14
                            maximumLength: 20
    						validator: RegExpValidator { regExp: /^[a-zA-Z0-9]*$/ }
                            onEnterKeyPress:{
                                loginVM.updateIdleTimer()
                                if(txt_password_old.text === thisItem._currentPw )
                                {
                                    stackviewLogin.changePassClear = txt_password_new.text
                                    stackviewLogin.changePassConfirmClear = txt_password_new_confirm.text
                                    internalLoginFunction.checkPasswordChange(txt_password_new.text, txt_password_new_confirm.text)

                                }
                                else
                                {
                                    _clearTextField = false;

                                    var component = Qt.createComponent("../Dialog/ModalDialogBox.qml");
                                    if (component.status === Component.Ready) {
                                        var messChangePasswordDialog = component.createObject(mainWindow,{_typeDialog:-1,
                                                                                        _content:"Current password incorrect",
                                                                                        _detailContent:"",
                                                                                        _isFunctional: false
                                                                                    });
                                        //messChangePasswordDialog.sendNotify()
                                        messChangePasswordDialog.open();
                                        if (thisItem._clearTextField){
                                            stackviewLogin.changePassClear = ""
                                            stackviewLogin.changePassConfirmClear = ""
                                        }
                                        else{
                                            stackviewLogin.changePassClear  = stackviewLogin.changePassClear
                                            stackviewLogin.changePassConfirmClear = stackviewLogin.changePassConfirmClear
                                        }
                                    }
                                }
                            }
                        }
                    }
                    Rectangle{
                        anchors.right: txt_password_new_confirm.parent.right
                        anchors.rightMargin: 5
                        color: "transparent"
                        width: parent.width*0.1
                        height: parent.height
                        Image {
                            anchors.fill: parent
                            property bool passwordVisible: false
                            source: passwordVisible ? "../../Resources/eye_open_black.png" : "../../Resources/eye_close_black.png"
                            MouseArea{
                                anchors.fill: parent
                                onClicked: {
                                    loginVM.updateIdleTimer()
                                    parent.passwordVisible = ! parent.passwordVisible
                                    if (parent.passwordVisible) txt_password_new_confirm.echoMode = TextInput.Normal
                                    else txt_password_new_confirm.echoMode = TextInput.Password
                                }
                            }
                        }
                    }
                }
                SpaceV{_space:25}
                Row{
                    height:parent.height*0.2 - 10
                    width:parent.width
                    anchors{
                        left:parent.left
                    }
                    ButtonText{
                        width:parent.width*0.5
                        height:35
                        _text:"Change password"
                        _btnColorMouseOver:"deepskyblue"
                        onClicked:{
                            loginVM.updateIdleTimer()
                            if(txt_password_old.text === thisItem._currentPw )
                            {
                                stackviewLogin.changePassClear = txt_password_new.text
                                stackviewLogin.changePassConfirmClear = txt_password_new_confirm.text
                                internalLoginFunction.checkPasswordChange(txt_password_new.text, txt_password_new_confirm.text)
                            }
                            else
                            {
                                _clearTextField = false;

                                var component = Qt.createComponent("../Dialog/ModalDialogBox.qml");
                                if (component.status === Component.Ready) {
                                var messChangePasswordDialog = component.createObject(mainWindow,{_typeDialog:-1,
                                                                                _content:"Current password incorrect",
                                                                                _detailContent:"",
                                                                                _isFunctional: false
                                                                                });
                                //messChangePasswordDialog.sendNotify()
                                messChangePasswordDialog.open();
                                if (thisItem._clearTextField){
                                    stackviewLogin.changePassClear = ""
                                    stackviewLogin.changePassConfirmClear = ""
                                }
                                else{
                                    stackviewLogin.changePassClear  = stackviewLogin.changePassClear
                                    stackviewLogin.changePassConfirmClear = stackviewLogin.changePassConfirmClear
                                }
                                }
                            }
                        }
                    }
                    SpaceH{_space:parent.height*0.125}
                    ButtonText{
                        width:parent.width*0.375
                        height:35
                        _text:"Clear"
                        _btnColorMouseOver:"deepskyblue"
                        onClicked:{
                            loginVM.updateIdleTimer()
                            txt_password_old.text = ""
                            txt_password_new.text = ""
                            txt_password_new_confirm.text = ""
                        }
                    }
                }
            }
        }
    }

    Component{  /* Delete User Account*/
        id:deleteUserAccountPanel
        Rectangle{
            color:"transparent"
            Column{
                height:parent.height
                width: parent.width*0.8
                anchors{
                    leftMargin:10
                    rightMargin:parent.width*0.1
                    horizontalCenter: parent.horizontalCenter
                }
                SpaceV{_space:94}
                Text{
                    height:contentHeight
                    width: contentWidth
                    text:"Delete User Account"
                    color:"deepskyblue"
                    font.pointSize:18
                    font.bold:true
                    horizontalAlignment:Text.AlignHCenter
                    verticalAlignment:Text.AlignVCenter
                }
                SpaceV{_space:39}
                Row{
                    height:parent.height*0.1
                    width: parent.width
                    anchors{
                        left:parent.left
                        leftMargin: parent.width * 0.1
                    }
                    Text{
                        text:"Account User"
                        width:parent.width*0.4
                        color:"Gold"
                        font.pointSize:12
                        font.bold:true
                        font.family:"Adobe Gothic Std B"
                        verticalAlignment: Text.AlignVCenter
                        anchors.verticalCenter: parent.verticalCenter
                    }
                    SpaceH{_space: 37}
                    Text{
                        text:"Level"
                        color:"Gold"
                        font.bold:true
                        width:parent.width*0.4
                        font.pointSize:12
                        font.family:"Adobe Gothic Std B"
                        verticalAlignment: Text.AlignVCenter
                        anchors.verticalCenter: parent.verticalCenter
                    }
                }
                ListView{
					id:listAccount

					property bool applyLastCheck: false
					property var listLastCheck: []

                    width:parent.width
                    height:parent.height*0.4
                    model: LoginModel{
                        list: loginVM
                    }
                    spacing: 4
                    anchors.margins: 10
                    clip: true
                    interactive: true
                    flickableDirection: Flickable.VerticalFlick
                    boundsBehavior: Flickable.StopAtBounds
                    section.property: "size"
                    section.criteria: ViewSection.FullString
                    ScrollBar.vertical: ScrollBar {
                        height:100
                        active: true
                        policy: ScrollBar.AlwaysOn
                        snapMode:ScrollBar.SnapOnRelease
                    }
                    visible:true
                    delegate:Rectangle {
                        color:"#222222"
                        height:30
                        width:parent.width
                        radius:4
                        scale:  mouseArea.containsMouse ? 1 : 1
                        smooth: mouseArea.containsMouse
                        opacity: enabled? 1:0.5
                        enabled:  {
                            if (model.username === "Admin") {
                                    return false;
                            }
                            if (_level === "Operator" && (model.level === "Operator" || model.level === "Technician" || model.level === "Administrator")) {
                                return false
                            } else if (_level === "Technician" && (model.level === "Technician" || model.level === "Administrator")) {
                                return false
                            } else if (_level === "Administrator"&& model.level === "Administrator") {
                                if (_username === "Admin") {
                                    return true
                                }
                                else {
                                return false
                                }
                            }
                            else {
                                return true
                            }
                        }
                        MouseArea{
                            hoverEnabled: true
                            id:mouseArea
                            acceptedButtons: Qt.LeftButton | Qt.RightButton
                            onClicked:{
                                loginVM.updateIdleTimer()
                                if(mouse.button === Qt.LeftButton){
                                }
                            }
                            anchors.fill: parent
                        }
                        Row{
                            width:parent.width
                            height: parent.height
                            anchors{
                                verticalCenter: parent.verticalCenter
                                left:parent.left
                                leftMargin:5
                                right:parent.right
                                rightMargin:5
                            }
                            CheckBox{
                                width:parent.width*0.2
                                anchors.verticalCenter: parent.verticalCenter
                                checked:model.checked
                                onCheckedChanged: {
                                    loginVM.updateIdleTimer()
                                    if(checked==false){
                                       model.checked=false
                                    }else{
                                        model.checked=true
                                    }
                                }
                            }
                            Text{
                                width:parent.width*0.4
                                text:model.username
                                color:"white"
                                font.pointSize:12
                                font.family:"Adobe Gothic Std B"
                                verticalAlignment: Text.AlignVCenter
                                anchors.verticalCenter: parent.verticalCenter
                            }
                            Text{
                                text:model.level
                                width:parent.width*0.4
                                color:"white"
                                font.pointSize:12
                                font.family:"Adobe Gothic Std B"
                                verticalAlignment: Text.AlignVCenter
                                anchors.verticalCenter: parent.verticalCenter
                            }
                        }
                    }
                }
                SpaceV{_space:10}
                Row{
                    height:parent.height*0.2 - 10
                    width:parent.width
                    anchors{
                        left:parent.left
                        leftMargin:20
                        right:parent.right
                        rightMargin:10
                    }
                    ButtonText{
                        width:parent.width*0.45
                        height:35
                        _text:"Delete"
                        _btnColorMouseOver:"deepskyblue"
                        onClicked:{
                                loginVM.updateIdleTimer()
                                internalLoginFunction.checkDeleteAccount()
                            }
                    }
                    SpaceH{_space:parent.height*0.1}
                }
            }
        }
    }

    /* Contruct LoginUser*/

    Rectangle{
        anchors.fill: parent
        width:parent.width
        height:parent.height
        radius:7
        border.color:"#322F2E"
        border.width:1
        gradient: Gradient {
            GradientStop { position: 0.0; color: "#1E1E1E" }
            GradientStop { position: 1.0; color: "#1E1E1E" }
        }
        Row{
            id:view1
            height:parent.height
            width:parent.width
            Column{
                width:parent.width/2
                height:parent.height
                Rectangle{
                    width:parent.width
                    height:parent.height*0.275
                    color: "transparent"
                    StackView{
                        anchors.fill: parent
                        initialItem: header_view
                    }
                }
                Rectangle{
                    width:parent.width
                    height:parent.height*0.225
                    color: "transparent"
                    StackView{
                        id: stackViewInforLogin
                        anchors.fill: parent
                        initialItem:inforLogin_view
                    }
                }
                Rectangle{
                    width:parent.width
                    height:parent.height*0.5
                    color: "transparent"
                    StackView{
                        anchors.fill: parent
                        initialItem:funtion_view
                    }
                }
            }
            Rectangle{
                id:sperator
                width:2
                height:parent.height*0.9
                color:"gray"
                anchors.verticalCenter: parent.verticalCenter
            }
            Rectangle{
                width:parent.width/2
                height:parent.height
                radius: 10
                anchors.margins: 10
                color: "transparent"
                StackView{
                    id:stackviewLogin
                    property string loginUserClear: ""
                    property string loginPassClear: ""
                    property string createUserClear: ""
                    property string createPassClear: ""
                    property string createPassconfirmClear: ""
                    property string changePassClear: ""
                    property string changePassConfirmClear: ""
                    property bool enable_mouseAction_function_view: true
                    property bool remember_account: false

                    anchors.fill: parent
                    focus: true
                }
            }
        }
    }

    Connections{
        target: loginVM
        ignoreUnknownSignals: true
        onAutoLogoutCheck:{
            internalLoginFunction.checkUserLogout()
        }
        onLoginCheck:{
            _codeLoginCheck = error_code
        }
        onUserDataBase:{
            _username = user
            _level = level
        }
        onCurrentPasswordCheck:
        {
            thisItem._currentPw = currentPassword

        }
        onChangePasswordCheck:{
           _codeChangePasswordCheck = error_code
        }
        onCreateNewAccountCheck:{
            _codeCreateAccountCheck = error_code
        }
        onDeleteAccountCheck:{
            _codeDeleteAccountCheck = error_code
        }
        onRememberAccountCheck:{
            stackviewLogin.loginUserClear = userRemember
            stackviewLogin.loginPassClear = "******"
            stackviewLogin.remember_account = true
        }
    }
}
