
function openForm(name) {
  var nrow;
  if (name === 'west') {
    nrow = 4;
  } else {
    nrow = 2;
  }
  document.getElementById("form_h3").innerHTML = 
    document.getElementById("pname").value =
      document.getElementById("saved_values").rows[nrow].cells.item(0).innerHTML;

  document.getElementById("mode").value =
    document.getElementById("saved_values").rows[nrow].cells.item(2).innerHTML;
  document.getElementById("period").value =
    document.getElementById("saved_values").rows[nrow].cells.item(3).innerHTML;
  document.getElementById("nightb").value =
    document.getElementById("saved_values").rows[nrow].cells.item(4).innerHTML;
  document.getElementById("nighte").value =
    document.getElementById("saved_values").rows[nrow].cells.item(5).innerHTML;
  document.getElementById("d_up").value =
    document.getElementById("saved_values").rows[nrow].cells.item(6).innerHTML;
  document.getElementById("d_down").value =
    document.getElementById("saved_values").rows[nrow].cells.item(7).innerHTML;
  document.getElementById("sclk").value =
    document.getElementById("saved_values").rows[nrow].cells.item(8).innerHTML;
  document.getElementById("lock").value =
    document.getElementById("saved_values").rows[nrow].cells.item(1).innerHTML;

  document.getElementById("popupForm").style.display = "block";
}

function closeForm() {
  document.getElementById("popupForm").style.display = "none";
}


function GetBLEData() {
  var nrow_d = 2;
  var nrow_s = 2;

  fetch(window.location.protocol + '//' + window.location.host + '/display_val')
  .then(response => response.json())
  .then(data => {
    console.log(data) // Prints result from `response.json()` in getRequest
    if ('name' in data) {
      if (data.name === 'EAST') {
        nrow_d = 2;
        nrow_s = 2;
      } else {
        nrow_d = 3;
        nrow_s = 4;
      }
      
      if ('tm' in data) {
        document.getElementById("curr_values").rows[nrow_d].cells.item(1).innerHTML = parseFloat(data.tm);
      }
      if ('tx' in data) {
        document.getElementById("curr_values").rows[nrow_d].cells.item(2).innerHTML = parseFloat(data.tx);  
      }
      if ('ta' in data) {
        document.getElementById("curr_values").rows[nrow_d].cells.item(3).innerHTML = parseFloat(data.ta);  
      }
      if ('pr' in data) {
        document.getElementById("curr_values").rows[nrow_d].cells.item(4).innerHTML = parseFloat(data.pr);  
      }
      if ('on' in data) {
        document.getElementById("curr_values").rows[nrow_d].cells.item(5).innerHTML = parseInt(data.on);  
      }
      if ('tup' in data) {
        document.getElementById("curr_values").rows[nrow_d].cells.item(6).innerHTML = data.tup;  
      }
      if ('ct' in data) {
        document.getElementById("curr_values").rows[nrow_d].cells.item(7).innerHTML = data.ct;  
      }

      if ('slk' in data) {
        document.getElementById("saved_values").rows[nrow_s].cells.item(1).innerHTML = 
          parseInt(data.slk);  
      }
      if ('rlk' in data) {
        document.getElementById("saved_values").rows[nrow_s + 1].cells.item(0).innerHTML = 
          parseInt(data.rlk);  
      }
      if ('smod' in data) {
        document.getElementById("saved_values").rows[nrow_s].cells.item(2).innerHTML = 
          parseInt(data.smod);  
      }
      if ('rmod' in data) {
        document.getElementById("saved_values").rows[nrow_s + 1].cells.item(1).innerHTML = 
          parseInt(data.rmod);  
      }
      if ('sper' in data) {
        document.getElementById("saved_values").rows[nrow_s].cells.item(3).innerHTML = 
          data.sper;  
      }
      if ('rper' in data) {
        document.getElementById("saved_values").rows[nrow_s + 1].cells.item(2).innerHTML = 
          data.rper;  
      }
      if ('snb' in data) {
        document.getElementById("saved_values").rows[nrow_s].cells.item(4).innerHTML = 
          data.snb;  
      }
      if ('rnb' in data) {
        document.getElementById("saved_values").rows[nrow_s + 1].cells.item(3).innerHTML = 
          data.rnb;  
      }
      if ('sne' in data) {
        document.getElementById("saved_values").rows[nrow_s].cells.item(5).innerHTML = 
          data.sne;  
      }
      if ('rne' in data) {
        document.getElementById("saved_values").rows[nrow_s + 1].cells.item(4).innerHTML = 
          data.rne;  
      }
      if ('sdup' in data) {
        document.getElementById("saved_values").rows[nrow_s].cells.item(6).innerHTML = 
          parseInt(data.sdup);  
      }
      if ('rdup' in data) {
        document.getElementById("saved_values").rows[nrow_s + 1].cells.item(5).innerHTML = 
          parseInt(data.rdup);  
      }
      if ('sddn' in data) {
        document.getElementById("saved_values").rows[nrow_s].cells.item(7).innerHTML = 
          parseInt(data.sddn);  
      }
      if ('rddn' in data) {
        document.getElementById("saved_values").rows[nrow_s + 1].cells.item(6).innerHTML = 
          parseInt(data.rddn);  
      }
      if ('ssclk' in data) {
        document.getElementById("saved_values").rows[nrow_s].cells.item(8).innerHTML = 
          parseInt(data.ssclk);  
      }
      if ('rsclk' in data) {
        document.getElementById("saved_values").rows[nrow_s + 1].cells.item(7).innerHTML = 
          parseInt(data.rsclk);  
      }
    }
  })
  .catch(error => {
    console.error(error);
  });

  setTimeout('GetBLEData()', 2000);
}

function start() {
  GetBLEData();
}

//
