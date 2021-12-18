<%@ page language="java" contentType="text/html; charset=gb2312" pageEncoding="gb2312"%>
<html>
<body>
                                                                                  欢迎来到PXXY影视窗口
   
	<%
	String name=(String)session.getAttribute("name");
	     out.print("尊敬的用户:"+name);
	%>
	 <script type="text/javascript">
	 
	       function loadPage(){
	         var targetURL=document.getElementById("url").value;
	         console.log(targetURL);
	         document.getElementById('iframePosition').src=targetURL; 
	       }
	        </script> 
	        
	        <div>
	        <p>请输入要加载的地址：<span id="currentTime"></span></p>
	         <p> 
	         <input type="text"id="url" value="https://www.bilibili.com/"> 
	         <input type="button" value="提交" onclick="loadPage()">
	          </p>
	            </div>
	            
	        <div>
	        <h3> 加载页面的位置: </h3> 
	        
       <iframe style="width: 100%; height:500px" id="iframePosition" src="https://www.bilibili.com/"></iframe> 
             
     </div>
</body>
</html>