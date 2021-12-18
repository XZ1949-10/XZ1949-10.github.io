<%@ page language="java" contentType="text/html; charset=UTF-8" pageEncoding="UTF-8"%>

<!DOCTYPE html PUBLIC "-//W3C//DTD HTML 4.01 Transitional//EN" "http://www.w3.org/TR/html4/loose.dtd">
<html>
<head>
<meta http-equiv="Content-Type" content="text/html; charset=UTF-8">
<title>登录/注册界面</title>
  <link rel = "stylesheet" href = "https://cdnjs.cloudflare.com/ajax/libs/normalize/5.0.0/normalize.min.css">
  <link rel = "stylesheet" href = "css/style.css">

</head>
<body>
<div class = "container right-panel-active">
	<!--注册-->
	<div class = "container__form container--signup">
		<form action = "Serlet_in_up" class = "form" id = "form1" method="post">
			<h2 class = "form__title">注册</h2>
			<input type = "text" placeholder = "User" class = "input" name="text" required /><span id="sasda"></span><br>
			<input type = "email" placeholder = "Email" id="acc"  class = "input" name="email" required /><span id="spa"></span><br>
			<input type = "password" placeholder = "Password" class = "input" name="password" required /><span id="spasddda"></span><br>
			<button type="submit" class = "btn">Sign Up</button>
		</form>
	</div>
	<!--登录--> 
	<div class = "container__form container--signin">
		<form action = "Serlet_in_up" class = "form" id = "form2" method="post">
			<h2 class = "form__title">登录</h2>
			<input type = "email" placeholder = "Email" class = "input" name="email"  required /><span id="passdsadw"></span><br>
			<input type = "password" placeholder = "Password" id="pass"  name="password" class = "input" required /><span id="passw"></span><br>
			<a href = "Login.jsp" class = "link">forget your password?</a>
			<button type="submit" class = "btn">Sign In</button>
		</form>
	</div>
	
	<!--覆盖之前的界面--> 
	<div class = "container__overlay">
		<div class = "overlay">
			<div class = "overlay__panel overlay--left">
				<button class = "btn" id = "signIn">Sign In</button>
			</div>
			<div class = "overlay__panel overlay--right">
				<button class = "btn" id = "signUp">Sign Up</button>
			</div>
		</div>
	</div>
	
</div>
  <script src = "js/in_up.js"></script>
</body>
</html>