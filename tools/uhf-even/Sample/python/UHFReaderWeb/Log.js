var Log = {
    getDate() {
        var date = new Date();
        var year = date.getFullYear();
        var month = date.getMonth() + 1;
        var day = date.getDate();
        var hour = date.getHours();
        var minute = date.getMinutes();
        var second = date.getSeconds();
        var sss = date.getMilliseconds()
        return [year, '/', month, '/', day, ' ', hour, ':', minute, ':', second, ':', sss, ' '].join('');
    },
    setLog(msg) {
        $('#log').append("<li>" + msg + "</li>")
    },
    setError(msg) {
        $('#log').append("<li>" + this.getDate() + "error：" + msg + "</li>")
    },
    setInfo(msg) {
        $('#log').append("<li>" + this.getDate() + "info：" + msg + "</li>")
    },
    setWarning(msg) {
        $('#log').append("<li>" + this.getDate() + "warning：" + msg + "</li>")
    },
    alertLog(msg) {
        alert(msg)
    }
}