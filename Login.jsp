<%@ page language="java" contentType="text/html; charset=gb2312" pageEncoding="gb2312"%>
<html>
<body>
                                                                                  ��ӭ����PXXYӰ�Ӵ���
   
	<%
	String name=(String)session.getAttribute("name");
	     out.print("�𾴵��û�:"+name);
	%>
	 <script type="text/javascript">
	 
	       function loadPage(){
	         var targetURL=document.getElementById("url").value;
	         console.log(targetURL);
	         document.getElementById('iframePosition').src=targetURL; 
	       }
	        </script> 
	        
	        <div>
	        <p>������Ҫ���صĵ�ַ��<span id="currentTime"></span></p>
	         <p> 
	         <input type="text"id="url" value="https://www.bilibili.com/"> 
	         <input type="button" value="�ύ" onclick="loadPage()">
	          </p>
	            </div>
	            
	        <div>
	        <h3> ����ҳ���λ��: </h3> 
	        
       <iframe style="width: 100%; height:500px" id="iframePosition" src="https://www.bilibili.com/"></iframe> 
             
     </div>
</body>
</html>