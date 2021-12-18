// 设置按钮 
const signInBtn = document.getElementById("signIn");
const signUpBtn = document.getElementById("signUp");
const fistForm = document.getElementById("form1");
const secondForm = document.getElementById("form2");
const container = document.querySelector(".container");
// 点击事件 
signInBtn.addEventListener("click", () => {
	container.classList.remove("right-panel-active");
});
signUpBtn.addEventListener("click", () => {
	container.classList.add("right-panel-active");
});


function createXMLHttpRequest() {
	var xmlHttp;
	// 适用于大多数浏览器，以及IE7和IE更高版本
	try{
		xmlHttp = new XMLHttpRequest();
	} catch (e) {
		// 适用于IE6
		try {
			xmlHttp = new ActiveXObject("Msxml2.XMLHTTP");
		} catch (e) {
			// 适用于IE5.5，以及IE更早版本
			try{
				xmlHttp = new ActiveXObject("Microsoft.XMLHTTP");
			} catch (e){}
		}
	}			
	return xmlHttp;
} 


window.onload = function() {
// 获取文本框，给它的失去焦点事件注册监听
var userEle = document.getElementById("acc");
userEle.onblur = function() {
	//1.得到异步对象
	var xmlHttp = createXMLHttpRequest();
	//2.打开连接
	xmlHttp.open("POST", "/JavaFinally/AjaxServerlet", true);
	//3.设置请求头：Content-Type
	xmlHttp.setRequestHeader("Content-Type", "application/x-www-form-urlencoded");
	//4.发送请求，给出请求体
	xmlHttp.send("email=" + userEle.value);
	
	//5.给xmlHttp的onreadystatechange事件注册监听
	xmlHttp.onreadystatechange = function() {
		if(xmlHttp.readyState == 4 && xmlHttp.status == 200) {//双重判断
			//获取服务器的响应，判断是否为1
			// 是：获取span，添加内容：“用户名已被注册”
			var text = xmlHttp.responseText;
			var span = document.getElementById("spa");
			if(text == "1" ) {
				//得到span元素
				 span.innerHTML = "用户名已被注册！";

			} 
			if (text == "0" ){
				 span.innerHTML = "用户名可用";
			}
					}
	};
};
};

